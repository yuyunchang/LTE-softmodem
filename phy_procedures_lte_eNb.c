/*******************************************************************************
    OpenAirInterface 
    Copyright(c) 1999 - 2014 Eurecom

    OpenAirInterface is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.


    OpenAirInterface is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with OpenAirInterface.The full GNU General Public License is 
   included in this distribution in the file called "COPYING". If not, 
   see <http://www.gnu.org/licenses/>.

  Contact Information
  OpenAirInterface Admin: openair_admin@eurecom.fr
  OpenAirInterface Tech : openair_tech@eurecom.fr
  OpenAirInterface Dev  : openair4g-devel@eurecom.fr
  
  Address      : Eurecom, Campus SophiaTech, 450 Route des Chappes, CS 50193 - 06904 Biot Sophia Antipolis cedex, FRANCE

 *******************************************************************************/

/*! \file phy_procedures_lte_eNB.c
 * \brief Implementation of eNB procedures from 36.213 LTE specifications
 * \author R. Knopp, F. Kaltenberger, N. Nikaein
 * \date 2011
 * \version 0.1
 * \company Eurecom
 * \email: knopp@eurecom.fr,florian.kaltenberger@eurecom.fr,navid.nikaein@eurecom.fr
 * \note
 * \warning
 */
#include "rt_wrapper.h"
#include "PHY/defs.h"
#include "PHY/extern.h"
#include "MAC_INTERFACE/defs.h"
#include "MAC_INTERFACE/extern.h"
#include "SCHED/defs.h"
#include "SCHED/extern.h"

#ifdef EMOS
#include "SCHED/phy_procedures_emos.h"
#endif

#define DEBUG_PHY_PROC
//#define DEBUG_ULSCH

//#ifdef OPENAIR2
#include "LAYER2/MAC/extern.h"
#include "LAYER2/MAC/defs.h"
#include "UTIL/LOG/log.h"
#include "UTIL/LOG/vcd_signal_dumper.h"
//#endif

#include "assertions.h"

#if defined(ENABLE_ITTI)
#   include "intertask_interface.h"
#   if defined(ENABLE_RAL)
#     include "timer.h"
#   endif
#endif

//#define DIAG_PHY

// =============== FAPI add by Jenny=======================
#include "maca_phy_if.h"
/* struct for MACA to get DL subframe configuration messages */
extern tMacaTxVectorReq sTxVectorReq[10];
extern tPhyDlschData sTxDataReq[10];
extern tMacaRxVectorReq sRxVectorReq[10];  
extern tMacaRxDataInd   sRxDataInd[10];
extern bool CrcPresentFlag[10];
extern bool RxPresentFlag[10];
extern tMacaHiDciVectorReq   sHiDciVectorReq[10];
extern bool SrPresentFlag[10];
extern bool HarqPresentFlag[10];
//extern int16_t Vr,Vi;  //by Jenny Vr, Vi vectors for TA estimation, by Jenny 20150601

int ulsch_FAPI_en=1;       //if not define ulsch_FAPI , please set=0 
#define REORDERPDU 1 //20150916  joe Rxvector pdu reoder
//-------Test bench Setting----------------------
int en_ULSCH_TESTBENCH=0;    //enable testbench, if IQ is from file, please defined ULSCH_TESTBENCH
int start_subframe=0;      //force parameters from start subframe
int force_ulsch_do_decode=0;  //1: pusch is forced to execute or it will be controlled by mac
int en_estTA=0;  //TA estimation at PUSCH, by Jenny 20150601

#define MEASURETXTIME //joe 20160427 Measure the execution time of TX
#define MEASURERXTIME //joe 20160427 Measure the execution time of RX
#define PRACHTHREAD // joe 20160517 create thread for PRACH
//------Test bench setting end-------------------


// Table 8.6.3-3 36.213
extern uint16_t beta_cqi[16];

// Table 8.6.3-2 36.213
extern uint16_t beta_ri[16];   //reserved

// Table 8.6.3-2 36.213
extern uint16_t beta_ack[16];//126.00
//=============FAPI End===================================

// ==============Time ==========================
#include <time.h>
#define MS 1E6
struct timespec requestStart, requestEnd;
struct timespec encoding_start, encoding_end;
struct timespec modulation_start, modulation_end;
// ============================================


	
	//////////////////////////////////////////////////////////


#define NS_PER_SLOT 500000

#define PUCCH 
#define PUCCH1_THRES 15 //joe 20160427 PUCCH SR Threshold 33->15
#define PUCCH1a_THRES 0 //wewe0812

int prach_counter = 0;

extern inline unsigned int taus(void);
extern int exit_openair;
//extern void do_OFDM_mod(mod_sym_t **txdataF, int32_t **txdata, uint32_t frame, uint16_t next_slot, LTE_DL_FRAME_PARMS *frame_parms);


unsigned char dlsch_input_buffer[2700] __attribute__ ((aligned(16)));
int eNB_sync_buffer0[640*6] __attribute__ ((aligned(16)));
int eNB_sync_buffer1[640*6] __attribute__ ((aligned(16)));
int *eNB_sync_buffer[2] = {eNB_sync_buffer0, eNB_sync_buffer1};

extern uint16_t hundred_times_log10_NPRB[100];

unsigned int max_peak_val; 
int max_sect_id, max_sync_pos;

//DCI_ALLOC_t dci_alloc[8];

#ifdef EMOS
fifo_dump_emos_eNB emos_dump_eNB;
#endif

#if defined(SMBV) && !defined(EXMIMO)
extern const char smbv_fname[];
extern unsigned short config_frames[4];
extern uint8_t smbv_frame_cnt;
#endif

#ifdef DIAG_PHY
extern int rx_sig_fifo;
#endif
static unsigned char I0_clear = 1;

uint8_t is_SR_subframe(PHY_VARS_eNB *phy_vars_eNB,uint8_t UE_id,uint8_t sched_subframe) {

  int subframe = phy_vars_eNB->proc[sched_subframe].subframe_rx;
  int frame = phy_vars_eNB->proc[sched_subframe].frame_rx;
  
  LOG_D(PHY,"[eNB %d][SR %x] Frame %d subframe %d Checking for SR TXOp(sr_ConfigIndex %d)\n",
	phy_vars_eNB->Mod_id,phy_vars_eNB->ulsch_eNB[UE_id]->rnti,frame,subframe,
	phy_vars_eNB->scheduling_request_config[UE_id].sr_ConfigIndex);
  
  if (phy_vars_eNB->scheduling_request_config[UE_id].sr_ConfigIndex <= 4) {        // 5 ms SR period
    if ((subframe%5) == phy_vars_eNB->scheduling_request_config[UE_id].sr_ConfigIndex)
      return(1);
  }
  else if (phy_vars_eNB->scheduling_request_config[UE_id].sr_ConfigIndex <= 14) {  // 10 ms SR period
    if (subframe==(phy_vars_eNB->scheduling_request_config[UE_id].sr_ConfigIndex-5))
      return(1);
  }
  else if (phy_vars_eNB->scheduling_request_config[UE_id].sr_ConfigIndex <= 34) { // 20 ms SR period
    if ((10*(frame&1)+subframe) == (phy_vars_eNB->scheduling_request_config[UE_id].sr_ConfigIndex-15))
      return(1);
  }
  else if (phy_vars_eNB->scheduling_request_config[UE_id].sr_ConfigIndex <= 74) { // 40 ms SR period
    if ((10*(frame&3)+subframe) == (phy_vars_eNB->scheduling_request_config[UE_id].sr_ConfigIndex-35))
      return(1);
  }
  else if (phy_vars_eNB->scheduling_request_config[UE_id].sr_ConfigIndex <= 154) { // 80 ms SR period
    if ((10*(frame&7)+subframe) == (phy_vars_eNB->scheduling_request_config[UE_id].sr_ConfigIndex-75))
      return(1);
  }

  return(0);
}
 
int32_t add_ue(int16_t rnti, PHY_VARS_eNB *phy_vars_eNB) {
  uint8_t i;

#ifdef DEBUG_PHY_PROC
  LOG_I(PHY,"[eNB %d] Adding UE with rnti %x\n",phy_vars_eNB->Mod_id,rnti);
#endif
  for (i=0;i<NUMBER_OF_UE_MAX;i++) {
    if ((phy_vars_eNB->dlsch_eNB[i]==NULL) || (phy_vars_eNB->ulsch_eNB[i]==NULL)) {
      LOG_E(PHY,"Can't add UE, not enough memory allocated\n");
      return(-1);
    }
    else {
      if (phy_vars_eNB->eNB_UE_stats[i].crnti==0) {
	LOG_I(PHY,"UE_id %d associated with rnti %x\n",i, rnti);
	phy_vars_eNB->dlsch_eNB[i][0]->rnti = rnti;
	phy_vars_eNB->ulsch_eNB[i]->rnti = rnti;
	phy_vars_eNB->eNB_UE_stats[i].crnti = rnti;
	return(i);
      }
    }
  }
  return(-1);
}

int32_t remove_ue(uint16_t rnti, PHY_VARS_eNB *phy_vars_eNB, uint8_t abstraction_flag) {
  uint8_t i;

#ifdef DEBUG_PHY_PROC
  LOG_I(PHY,"eNB %d removing UE with rnti %x\n",phy_vars_eNB->Mod_id,rnti);
#endif
  for (i=0;i<NUMBER_OF_UE_MAX;i++) {
    if ((phy_vars_eNB->dlsch_eNB[i]==NULL) || (phy_vars_eNB->ulsch_eNB[i]==NULL)) {
      LOG_E(PHY,"Can't remove UE, not enough memory allocated\n");
      return(-1);
    }
    else {
      if (phy_vars_eNB->eNB_UE_stats[i].crnti==rnti) {
	//msg("[PHY] UE_id %d\n",i);
	clean_eNb_dlsch(phy_vars_eNB->dlsch_eNB[i][0], abstraction_flag);
	clean_eNb_ulsch(phy_vars_eNB->ulsch_eNB[i],abstraction_flag);
	//phy_vars_eNB->eNB_UE_stats[i].crnti = 0;
	memset(&phy_vars_eNB->eNB_UE_stats[i],0,sizeof(LTE_eNB_UE_stats));
	//	mac_exit_wrapper("Removing UE");	
	return(i);
      }
    }
  }

  return(-1);
}

int8_t find_next_ue_index(PHY_VARS_eNB *phy_vars_eNB) {
  uint8_t i;

  for (i=0;i<NUMBER_OF_UE_MAX;i++) {
    if (phy_vars_eNB->eNB_UE_stats[i].crnti==0) {
      /*if ((phy_vars_eNB->dlsch_eNB[i]) && 
	(phy_vars_eNB->dlsch_eNB[i][0]) && 
	(phy_vars_eNB->dlsch_eNB[i][0]->rnti==0))*/ 
      LOG_D(PHY,"Next free UE id is %d\n",i);
      return(i);
    }
  }
  return(-1);
}

int get_ue_active_harq_pid(uint8_t Mod_id,uint8_t CC_id,uint16_t rnti,int frame, uint8_t subframe,uint8_t *harq_pid,uint8_t *round,uint8_t ul_flag) {

  LTE_eNB_DLSCH_t *DLSCH_ptr;  
  LTE_eNB_ULSCH_t *ULSCH_ptr;  
  //  uint8_t subframe_m4;
  uint8_t ulsch_subframe,ulsch_frame; 
  uint8_t i;
  int8_t UE_id = find_ue(rnti,PHY_vars_eNB_g[Mod_id][CC_id]);
  //  int frame    = PHY_vars_eNB_g[Mod_id][CC_id]->proc[sched_subframe].frame_tx;
  //  int subframe = PHY_vars_eNB_g[Mod_id][CC_id]->proc[sched_subframe].subframe_tx;

  if (UE_id==-1) {
    LOG_D(PHY,"Cannot find UE with rnti %x (Mod_id %d, CC_id %d)\n",rnti, Mod_id, CC_id);
    *round=0;
    return(-1);
  }

  if (ul_flag == 0)  {// this is a DL request
    DLSCH_ptr = PHY_vars_eNB_g[Mod_id][CC_id]->dlsch_eNB[(uint32_t)UE_id][0];
    /*
#ifdef DEBUG_PHY_PROC
    LOG_D(PHY,"[eNB %d] get_ue_active_harq_pid: Frame %d subframe %d, current harq_id %d\n",
	  Mod_id,frame,subframe,DLSCH_ptr->harq_ids[subframe]);
#endif
    */
    // switch on TDD or FDD configuration here later
    *harq_pid = DLSCH_ptr->harq_ids[subframe];
    if ((*harq_pid<DLSCH_ptr->Mdlharq) && 
	((DLSCH_ptr->harq_processes[*harq_pid]->round > 0))) {

      *round = DLSCH_ptr->harq_processes[*harq_pid]->round;
      LOG_D(PHY,"round %d\n",*round);
    
      //    else if ((subframe_m4==5) || (subframe_m4==6)) {
      //      *harq_pid = 0;//DLSCH_ptr->harq_ids[subframe_m4];//Ankit
      //     *round    = DLSCH_ptr->harq_processes[*harq_pid]->round;
      //    }
    }
    else {
      // get first free harq_pid (i.e. round 0)
      for (i=0;i<DLSCH_ptr->Mdlharq;i++) {
	if (DLSCH_ptr->harq_processes[i]!=NULL) {
	  if (DLSCH_ptr->harq_processes[i]->status != ACTIVE) {
	    *harq_pid = i;//0;//i; //(Ankit)
	    *round = 0;
	    return(0);
	  }
	  else {
	    LOG_D(PHY,"process %d is active\n",i);
	  }
	}
	else {
	  LOG_E(PHY,"[eNB %d] DLSCH process %d for rnti %x (UE_id %d) not allocated\n",Mod_id,i,rnti,UE_id);
	  return(-1);
	}
      }
    }
  }
  else {  // This is a UL request

    ULSCH_ptr = PHY_vars_eNB_g[Mod_id][CC_id]->ulsch_eNB[(uint32_t)UE_id];
    ulsch_subframe = pdcch_alloc2ul_subframe(&PHY_vars_eNB_g[Mod_id][CC_id]->lte_frame_parms,subframe);
    ulsch_frame    = pdcch_alloc2ul_frame(&PHY_vars_eNB_g[Mod_id][CC_id]->lte_frame_parms,frame,subframe);
    // Note this is for TDD configuration 3,4,5 only
    *harq_pid = subframe2harq_pid(&PHY_vars_eNB_g[Mod_id][CC_id]->lte_frame_parms,
				  ulsch_frame,
				  ulsch_subframe);
    *round    = ULSCH_ptr->harq_processes[*harq_pid]->round;
    LOG_T(PHY,"[eNB %d][PUSCH %d] Frame %d subframe %d Checking HARQ, round %d\n",Mod_id,*harq_pid,frame,subframe,*round);
  }
  return(0);
}


int CCE_table[800];

void init_nCCE_table(void) {
  memset(CCE_table,0,800*sizeof(int));
}


int get_nCCE_offset(unsigned char L, int nCCE, int common_dci, unsigned short rnti, unsigned char subframe) {

  int search_space_free,m,nb_candidates = 0,l,i;
  unsigned int Yk;
  /*
    printf("CCE Allocation: ");
    for (i=0;i<nCCE;i++)
    printf("%d.",CCE_table[i]);
    printf("\n");
  */
  if (common_dci == 1) {
    // check CCE(0 ... L-1)
    nb_candidates = (L==4) ? 4 : 2;
    for (m = 0 ; m < nb_candidates ; m++) {
      search_space_free = 1;
      for (l=0;l<L;l++) {
	if (CCE_table[(m*L) + l] == 1) {
	  search_space_free = 0;
	  break;
	}
      }
      if (search_space_free == 1) {
	for (l=0;l<L;l++)
	  CCE_table[(m*L)+l]=1;
	return(m*L);
      }
    }
    return(-1);

  }
  else {  // Find first available in ue specific search space
          // according to procedure in Section 9.1.1 of 36.213 (v. 8.6)
    // compute Yk
    Yk = (unsigned int)rnti;

    for (i=0;i<=subframe;i++)
      Yk = (Yk*39827)%65537;

    Yk = Yk % (nCCE/L);


    switch (L) {
    case 1:
    case 2:
      nb_candidates = 6;
      break;
    case 4:
    case 8:
      nb_candidates = 2;
      break;
    default:
      DevParam(L, nCCE, rnti);
      break;
    }

    //    LOG_I(PHY,"rnti %x, Yk = %d, nCCE %d (nCCE/L %d),nb_cand %d\n",rnti,Yk,nCCE,nCCE/L,nb_candidates);

    for (m = 0 ; m < nb_candidates ; m++) {
      search_space_free = 1;
      for (l=0;l<L;l++) {
	if (CCE_table[(((Yk+m)%(nCCE/L))*L) + l] == 1) {
	  search_space_free = 0;
	  break;
	}
      }
      if (search_space_free == 1) {
	for (l=0;l<L;l++)
	  CCE_table[(((Yk+m)%(nCCE/L))*L)+l]=1;
	return(((Yk+m)%(nCCE/L))*L);
      }
    }
    return(-1);
  }
}

int16_t get_target_ul_rx_power(module_id_t module_idP, uint8_t CC_id) {
  //return PHY_vars_eNB_g[module_idP][CC_id]->PHY_measurements_eNB[0].n0_power_tot_dBm;
  return PHY_vars_eNB_g[module_idP][CC_id]->lte_frame_parms.ul_power_control_config_common.p0_NominalPUSCH;
}

#ifdef EMOS
void phy_procedures_emos_eNB_TX(unsigned char subframe, PHY_VARS_eNB *phy_vars_eNB) {

}
#endif

/*
  void phy_procedures_eNB_S_TX(unsigned char next_slot,PHY_VARS_eNB *phy_vars_eNB,uint8_t abstraction_flag) {

  int sect_id = 0, aa;

  if (next_slot%2==0) {
  #ifdef DEBUG_PHY_PROC
  msg("[PHY][eNB %d] Frame %d, slot %d: Generating pilots for DL-S\n",
  phy_vars_eNB->Mod_id,phy_vars_eNB->frame,next_slot);
  #endif
    
  for (sect_id=0;sect_id<number_of_cards;sect_id++) {
  if (abstraction_flag == 0) {

  for (aa=0; aa<phy_vars_eNB->lte_frame_parms.nb_antennas_tx; aa++) {
	  
	  
  #ifdef IFFT_FPGA
  memset(&phy_vars_eNB->lte_eNB_common_vars.txdataF[sect_id][aa][next_slot*(phy_vars_eNB->lte_frame_parms.N_RB_DL*12)*(phy_vars_eNB->lte_frame_parms.symbols_per_tti>>1)],
  0,(phy_vars_eNB->lte_frame_parms.N_RB_DL*12)*(phy_vars_eNB->lte_frame_parms.symbols_per_tti>>1)*sizeof(mod_sym_t));
  #else
  memset(&phy_vars_eNB->lte_eNB_common_vars.txdataF[sect_id][aa][next_slot*phy_vars_eNB->lte_frame_parms.ofdm_symbol_size*(phy_vars_eNB->lte_frame_parms.symbols_per_tti>>1)],
  0,phy_vars_eNB->lte_frame_parms.ofdm_symbol_size*(phy_vars_eNB->lte_frame_parms.symbols_per_tti>>1)*sizeof(mod_sym_t));
  #endif
  }

  generate_pilots_slot(phy_vars_eNB,
  phy_vars_eNB->lte_eNB_common_vars.txdataF[sect_id],
  AMP,
  next_slot);

  msg("[PHY][eNB] Frame %d, subframe %d Generating PSS\n",
  phy_vars_eNB->frame,next_slot>>1);
	
  generate_pss(phy_vars_eNB->lte_eNB_common_vars.txdataF[sect_id],
  4*AMP,
  &phy_vars_eNB->lte_frame_parms,
  2,
  next_slot);
      
  }
  else {
  #ifdef PHY_ABSTRACTION
  generate_pss_emul(phy_vars_eNB,sect_id);
  #endif
  }
  }
  }
  }
*/ 

void phy_procedures_eNB_S_RX(unsigned char sched_subframe,PHY_VARS_eNB *phy_vars_eNB,uint8_t abstraction_flag,relaying_type_t r_type)
{
  UNUSED(r_type);

  //  unsigned char sect_id=0; 
  int subframe = phy_vars_eNB->proc[sched_subframe].subframe_rx;
 
#ifdef DEBUG_PHY_PROC
  LOG_D(PHY,"[eNB %d] Frame %d: Doing phy_procedures_eNB_S_RX(%d)\n", phy_vars_eNB->Mod_id,phy_vars_eNB->proc[sched_subframe].frame_rx, subframe);
#endif    

  //  for (sect_id=0;sect_id<number_of_cards;sect_id++) {
    
  if (abstraction_flag == 0) {
    lte_eNB_I0_measurements(phy_vars_eNB,
			    0,
			    phy_vars_eNB->first_run_I0_measurements);
  }
#ifdef PHY_ABSTRACTION
  else {
    lte_eNB_I0_measurements_emul(phy_vars_eNB,
				 0);
  }
#endif

  
  if (I0_clear == 1)
    I0_clear = 0;
}



#ifdef EMOS
void phy_procedures_emos_eNB_RX(unsigned char subframe,PHY_VARS_eNB *phy_vars_eNB) {
  
  uint8_t aa;
  uint16_t last_subframe_emos;
  uint16_t pilot_pos1 = 3 - phy_vars_eNB->lte_frame_parms.Ncp, pilot_pos2 = 10 - 2*phy_vars_eNB->lte_frame_parms.Ncp;
  uint32_t bytes;

  last_subframe_emos=0;

#ifdef EMOS_CHANNEL
  //if (last_slot%2==1) // this is for all UL subframes
  if (subframe==3) 
    for (aa=0; aa<phy_vars_eNB->lte_frame_parms.nb_antennas_rx; aa++) {
      memcpy(&emos_dump_eNB.channel[aa][last_subframe_emos*2*phy_vars_eNB->lte_frame_parms.N_RB_UL*12],              
	     &phy_vars_eNB->lte_eNB_pusch_vars[0]->drs_ch_estimates[0][aa][phy_vars_eNB->lte_frame_parms.N_RB_UL*12*pilot_pos1],
	     phy_vars_eNB->lte_frame_parms.N_RB_UL*12*sizeof(int));
      memcpy(&emos_dump_eNB.channel[aa][(last_subframe_emos*2+1)*phy_vars_eNB->lte_frame_parms.N_RB_UL*12],          
	     &phy_vars_eNB->lte_eNB_pusch_vars[0]->drs_ch_estimates[0][aa][phy_vars_eNB->lte_frame_parms.N_RB_UL*12*pilot_pos2],
	     phy_vars_eNB->lte_frame_parms.N_RB_UL*12*sizeof(int));
    }
#endif

  if (subframe==4) {
    emos_dump_eNB.timestamp = rt_get_time_ns();
    emos_dump_eNB.frame_tx = phy_vars_eNB->proc[subframe].frame_rx;
    emos_dump_eNB.rx_total_gain_dB = phy_vars_eNB->rx_total_gain_eNB_dB;
    emos_dump_eNB.mimo_mode = phy_vars_eNB->transmission_mode[0];
    memcpy(&emos_dump_eNB.PHY_measurements_eNB,
           &phy_vars_eNB->PHY_measurements_eNB[0],
           sizeof(PHY_MEASUREMENTS_eNB));
    memcpy(&emos_dump_eNB.eNB_UE_stats[0],&phy_vars_eNB->eNB_UE_stats[0],NUMBER_OF_UE_MAX*sizeof(LTE_eNB_UE_stats));

    bytes = rtf_put(CHANSOUNDER_FIFO_MINOR, &emos_dump_eNB, sizeof(fifo_dump_emos_eNB));
    //bytes = rtf_put(CHANSOUNDER_FIFO_MINOR, "test", sizeof("test"));
    if (bytes!=sizeof(fifo_dump_emos_eNB)) {
      LOG_W(PHY,"[eNB %d] Frame %d, subframe %d, Problem writing EMOS data to FIFO (bytes=%d, size=%d)\n",
            phy_vars_eNB->Mod_id,phy_vars_eNB->proc[(subframe+1)%10].frame_rx, subframe,bytes,sizeof(fifo_dump_emos_eNB));
    }
    else {
      if (phy_vars_eNB->proc[(subframe+1)%10].frame_tx%100==0) {
        LOG_I(PHY,"[eNB %d] Frame %d (%d), subframe %d, Writing %d bytes EMOS data to FIFO\n",
              phy_vars_eNB->Mod_id,phy_vars_eNB->proc[(subframe+1)%10].frame_rx, ((fifo_dump_emos_eNB*)&emos_dump_eNB)->frame_tx, subframe, bytes);
      }
    }
  }
}
#endif

#ifndef OPENAIR2
void fill_dci(DCI_PDU *DCI_pdu, uint8_t sched_subframe, PHY_VARS_eNB *phy_vars_eNB) {

  int i;
  uint8_t cooperation_flag = phy_vars_eNB->cooperation_flag;
  uint8_t transmission_mode = phy_vars_eNB->transmission_mode[0];

  uint32_t rballoc = 0x7FFF;
  uint32_t rballoc2 = 0x000F;
  int subframe = phy_vars_eNB->proc[sched_subframe].subframe_tx;
  /*
    uint32_t rand = taus();
    if ((subframe==8) || (subframe==9) || (subframe==0))
    rand = (rand%5)+5;
    else
    rand = (rand%4)+5;
  */
  uint32_t bcch_pdu;
  uint64_t dlsch_pdu;

  DCI_pdu->Num_common_dci = 0;
  DCI_pdu->Num_ue_spec_dci=0;


  
  switch (subframe) {
  case 5:
    DCI_pdu->Num_common_dci = 1;
    DCI_pdu->dci_alloc[0].L          = 2;
    DCI_pdu->dci_alloc[0].rnti       = SI_RNTI;
    DCI_pdu->dci_alloc[0].format     = format1A;
    DCI_pdu->dci_alloc[0].ra_flag    = 0;
    switch (phy_vars_eNB->lte_frame_parms.N_RB_DL) {
    case 6:
      if (phy_vars_eNB->lte_frame_parms.frame_type == FDD) {
	DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1A_5MHz_FDD_t;
	((DCI1A_1_5MHz_FDD_t*)&bcch_pdu)->type              = 1;
	((DCI1A_1_5MHz_FDD_t*)&bcch_pdu)->vrb_type          = 0;
	((DCI1A_1_5MHz_FDD_t*)&bcch_pdu)->rballoc           = computeRIV(25,10,3);
	((DCI1A_1_5MHz_FDD_t*)&bcch_pdu)->ndi               = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	((DCI1A_1_5MHz_FDD_t*)&bcch_pdu)->rv                = 1;
	((DCI1A_1_5MHz_FDD_t*)&bcch_pdu)->mcs               = 1;
	((DCI1A_1_5MHz_FDD_t*)&bcch_pdu)->harq_pid          = 0;
	((DCI1A_1_5MHz_FDD_t*)&bcch_pdu)->TPC               = 1;      // set to 3 PRB
	memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],&bcch_pdu,sizeof(DCI1A_1_5MHz_TDD_1_6_t));	
      }
      else {
	DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1A_1_5MHz_TDD_1_6_t;
	((DCI1A_1_5MHz_TDD_1_6_t*)&bcch_pdu)->type              = 1;
	((DCI1A_1_5MHz_TDD_1_6_t*)&bcch_pdu)->vrb_type          = 0;
	((DCI1A_1_5MHz_TDD_1_6_t*)&bcch_pdu)->rballoc           = computeRIV(25,10,3);
	((DCI1A_1_5MHz_TDD_1_6_t*)&bcch_pdu)->ndi               = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	((DCI1A_1_5MHz_TDD_1_6_t*)&bcch_pdu)->rv                = 1;
	((DCI1A_1_5MHz_TDD_1_6_t*)&bcch_pdu)->mcs               = 1;
	((DCI1A_1_5MHz_TDD_1_6_t*)&bcch_pdu)->harq_pid          = 0;
	((DCI1A_1_5MHz_TDD_1_6_t*)&bcch_pdu)->TPC               = 1;      // set to 3 PRB
	memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],&bcch_pdu,sizeof(DCI1A_1_5MHz_TDD_1_6_t));	
      }
      break;
    case 25:
    default:
      if (phy_vars_eNB->lte_frame_parms.frame_type == FDD) {
	DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1A_5MHz_FDD_t;
	((DCI1A_5MHz_FDD_t*)&bcch_pdu)->type              = 1;
	((DCI1A_5MHz_FDD_t*)&bcch_pdu)->vrb_type          = 0;
	((DCI1A_5MHz_FDD_t*)&bcch_pdu)->rballoc           = computeRIV(25,10,3);
	((DCI1A_5MHz_FDD_t*)&bcch_pdu)->ndi               = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	((DCI1A_5MHz_FDD_t*)&bcch_pdu)->rv                = 1;
	((DCI1A_5MHz_FDD_t*)&bcch_pdu)->mcs               = 1;
	((DCI1A_5MHz_FDD_t*)&bcch_pdu)->harq_pid          = 0;
	((DCI1A_5MHz_FDD_t*)&bcch_pdu)->TPC               = 1;      // set to 3 PRB
	memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],&bcch_pdu,sizeof(DCI1A_5MHz_TDD_1_6_t));	
      }
      else {
	DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1A_5MHz_TDD_1_6_t;
	((DCI1A_5MHz_TDD_1_6_t*)&bcch_pdu)->type              = 1;
	((DCI1A_5MHz_TDD_1_6_t*)&bcch_pdu)->vrb_type          = 0;
	((DCI1A_5MHz_TDD_1_6_t*)&bcch_pdu)->rballoc           = computeRIV(25,10,3);
	((DCI1A_5MHz_TDD_1_6_t*)&bcch_pdu)->ndi               = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	((DCI1A_5MHz_TDD_1_6_t*)&bcch_pdu)->rv                = 1;
	((DCI1A_5MHz_TDD_1_6_t*)&bcch_pdu)->mcs               = 1;
	((DCI1A_5MHz_TDD_1_6_t*)&bcch_pdu)->harq_pid          = 0;
	((DCI1A_5MHz_TDD_1_6_t*)&bcch_pdu)->TPC               = 1;      // set to 3 PRB
	memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],&bcch_pdu,sizeof(DCI1A_5MHz_TDD_1_6_t));	
      }
      break;
    case 50:
      if (phy_vars_eNB->lte_frame_parms.frame_type == FDD) {
	DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1A_10MHz_FDD_t;
	((DCI1A_10MHz_FDD_t*)&bcch_pdu)->type              = 1;
	((DCI1A_10MHz_FDD_t*)&bcch_pdu)->vrb_type          = 0;
	((DCI1A_10MHz_FDD_t*)&bcch_pdu)->rballoc           = computeRIV(25,10,3);
	((DCI1A_10MHz_FDD_t*)&bcch_pdu)->ndi               = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	((DCI1A_10MHz_FDD_t*)&bcch_pdu)->rv                = 1;
	((DCI1A_10MHz_FDD_t*)&bcch_pdu)->mcs               = 1;
	((DCI1A_10MHz_FDD_t*)&bcch_pdu)->harq_pid          = 0;
	((DCI1A_10MHz_FDD_t*)&bcch_pdu)->TPC               = 1;      // set to 3 PRB
	memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],&bcch_pdu,sizeof(DCI1A_10MHz_TDD_1_6_t));	
      }
      else {
	DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1A_10MHz_TDD_1_6_t;
	((DCI1A_10MHz_TDD_1_6_t*)&bcch_pdu)->type              = 1;
	((DCI1A_10MHz_TDD_1_6_t*)&bcch_pdu)->vrb_type          = 0;
	((DCI1A_10MHz_TDD_1_6_t*)&bcch_pdu)->rballoc           = computeRIV(25,10,3);
	((DCI1A_10MHz_TDD_1_6_t*)&bcch_pdu)->ndi               = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	((DCI1A_10MHz_TDD_1_6_t*)&bcch_pdu)->rv                = 1;
	((DCI1A_10MHz_TDD_1_6_t*)&bcch_pdu)->mcs               = 1;
	((DCI1A_10MHz_TDD_1_6_t*)&bcch_pdu)->harq_pid          = 0;
	((DCI1A_10MHz_TDD_1_6_t*)&bcch_pdu)->TPC               = 1;      // set to 3 PRB
	memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],&bcch_pdu,sizeof(DCI1A_10MHz_TDD_1_6_t));	
      }
    break;
    case 100:
      if (phy_vars_eNB->lte_frame_parms.frame_type == FDD) {
	DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1A_10MHz_FDD_t;
	((DCI1A_20MHz_FDD_t*)&bcch_pdu)->type              = 1;
	((DCI1A_20MHz_FDD_t*)&bcch_pdu)->vrb_type          = 0;
	((DCI1A_20MHz_FDD_t*)&bcch_pdu)->rballoc           = computeRIV(25,10,3);
	((DCI1A_20MHz_FDD_t*)&bcch_pdu)->ndi               = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	((DCI1A_20MHz_FDD_t*)&bcch_pdu)->rv                = 1;
	((DCI1A_20MHz_FDD_t*)&bcch_pdu)->mcs               = 1;
	((DCI1A_20MHz_FDD_t*)&bcch_pdu)->harq_pid          = 0;
	((DCI1A_20MHz_FDD_t*)&bcch_pdu)->TPC               = 1;      // set to 3 PRB
	memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],&bcch_pdu,sizeof(DCI1A_20MHz_TDD_1_6_t));	
      }
      else {
	DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1A_20MHz_TDD_1_6_t;
	((DCI1A_20MHz_TDD_1_6_t*)&bcch_pdu)->type              = 1;
	((DCI1A_20MHz_TDD_1_6_t*)&bcch_pdu)->vrb_type          = 0;
	((DCI1A_20MHz_TDD_1_6_t*)&bcch_pdu)->rballoc           = computeRIV(25,10,3);
	((DCI1A_20MHz_TDD_1_6_t*)&bcch_pdu)->ndi               = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	((DCI1A_20MHz_TDD_1_6_t*)&bcch_pdu)->rv                = 1;
	((DCI1A_20MHz_TDD_1_6_t*)&bcch_pdu)->mcs               = 1;
	((DCI1A_20MHz_TDD_1_6_t*)&bcch_pdu)->harq_pid          = 0;
	((DCI1A_20MHz_TDD_1_6_t*)&bcch_pdu)->TPC               = 1;      // set to 3 PRB
	memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],&bcch_pdu,sizeof(DCI1A_20MHz_TDD_1_6_t));	
      }
    break;
    }
  case 6:
    /*
      DCI_pdu->Num_ue_spec_dci = 1;
      DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI2_5MHz_2A_M10PRB_TDD_t;
      DCI_pdu->dci_alloc[0].L          = 2;
      DCI_pdu->dci_alloc[0].rnti       = 0x1236;
      DCI_pdu->dci_alloc[0].format     = format2_2A_M10PRB;
      DCI_pdu->dci_alloc[0].ra_flag    = 0;

      DLSCH_alloc_pdu1.rballoc          = 0x00ff;
      DLSCH_alloc_pdu1.TPC              = 0;
      DLSCH_alloc_pdu1.dai              = 0;
      DLSCH_alloc_pdu1.harq_pid         = 0;
      DLSCH_alloc_pdu1.tb_swap          = 0;
      DLSCH_alloc_pdu1.mcs1             = 0;
      DLSCH_alloc_pdu1.ndi1             = 1;
      DLSCH_alloc_pdu1.rv1              = 0;
      DLSCH_alloc_pdu1.tpmi             = 0;
      memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],(void *)&DLSCH_alloc_pdu1,sizeof(DCI2_5MHz_2A_M10PRB_TDD_t));
    */
    break;
  case 7:
    DCI_pdu->Num_ue_spec_dci = 1;
    DCI_pdu->dci_alloc[0].L          = 2;
    DCI_pdu->dci_alloc[0].rnti       = 0x1235;
    DCI_pdu->dci_alloc[0].format     = format1;
    DCI_pdu->dci_alloc[0].ra_flag    = 0;
    
    if (transmission_mode<3) {
      //user 1
      switch (phy_vars_eNB->lte_frame_parms.N_RB_DL) {
      case 25:
	if (phy_vars_eNB->lte_frame_parms.frame_type == FDD) {
	  DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1_5MHz_FDD_t; 
	
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->rballoc          = rballoc;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->TPC              = 0;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->harq_pid         = 0;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->mcs              = openair_daq_vars.target_ue_dl_mcs;
	  //((DCI1_5MHz_FDD_t *)&dlsch_pdu)->mcs              = (unsigned char) ((phy_vars_eNB->frame%1024)%28);      
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->ndi              = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->rv               = 0;
	  memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],(void *)&dlsch_pdu,sizeof(DCI1_5MHz_TDD_t));
	  
	  /*
	  //user2
	  DCI_pdu->dci_alloc[1].dci_length = sizeof_DCI1_5MHz_TDD_t; 
	  DCI_pdu->dci_alloc[1].L          = 2;
	  DCI_pdu->dci_alloc[1].rnti       = 0x1236;
	  DCI_pdu->dci_alloc[1].format     = format1;
	  DCI_pdu->dci_alloc[1].ra_flag    = 0;
	  
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->rballoc          = rballoc2;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->TPC              = 0;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->dai              = 0;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->harq_pid         = 1;
	  //((DCI1_5MHz_FDD_t *)&dlsch_pdu)->mcs              = (unsigned char) ((phy_vars_eNB->proc[subframe].frame%1024)%28);      
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->mcs              = openair_daq_vars.target_ue_dl_mcs;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->ndi              = 1;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->rv               = 0;
	  memcpy((void*)&DCI_pdu->dci_alloc[1].dci_pdu[0],(void *)&((DCI1_5MHz_FDD_t *)&dlsch_pdu)->,sizeof(DCI1_5MHz_TDD_t));
	  */
	}
	else {
	  DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1_5MHz_TDD_t; 
	
	  ((DCI1_5MHz_TDD_t *)&dlsch_pdu)->rballoc          = rballoc;
	  ((DCI1_5MHz_TDD_t *)&dlsch_pdu)->TPC              = 0;
	  ((DCI1_5MHz_TDD_t *)&dlsch_pdu)->dai              = 0;
	  ((DCI1_5MHz_TDD_t *)&dlsch_pdu)->harq_pid         = 0;
	  ((DCI1_5MHz_TDD_t *)&dlsch_pdu)->mcs              = openair_daq_vars.target_ue_dl_mcs;
	  //((DCI1_5MHz_TDD_t *)&dlsch_pdu)->mcs              = (unsigned char) ((phy_vars_eNB->frame%1024)%28);      
	  ((DCI1_5MHz_TDD_t *)&dlsch_pdu)->ndi              = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	  ((DCI1_5MHz_TDD_t *)&dlsch_pdu)->rv               = 0;
	  memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],(void *)&dlsch_pdu,sizeof(DCI1_5MHz_TDD_t));
	  
	  /*
	  //user2
	  DCI_pdu->dci_alloc[1].dci_length = sizeof_DCI1_5MHz_TDD_t; 
	  DCI_pdu->dci_alloc[1].L          = 2;
	  DCI_pdu->dci_alloc[1].rnti       = 0x1236;
	  DCI_pdu->dci_alloc[1].format     = format1;
	  DCI_pdu->dci_alloc[1].ra_flag    = 0;
	  
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->rballoc          = rballoc2;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->TPC              = 0;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->dai              = 0;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->harq_pid         = 1;
	  //((DCI1_5MHz_FDD_t *)&dlsch_pdu)->mcs              = (unsigned char) ((phy_vars_eNB->proc[subframe].frame%1024)%28);      
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->mcs              = openair_daq_vars.target_ue_dl_mcs;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->ndi              = 1;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->rv               = 0;
	  memcpy((void*)&DCI_pdu->dci_alloc[1].dci_pdu[0],(void *)&((DCI1_5MHz_FDD_t *)&dlsch_pdu)->,sizeof(DCI1_5MHz_TDD_t));
	  */
	}
      
	break;
      case 50:

	if (phy_vars_eNB->lte_frame_parms.frame_type == FDD) {
	  DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1_10MHz_FDD_t; 
	
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->rballoc          = rballoc;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->TPC              = 0;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->harq_pid         = 0;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->mcs              = openair_daq_vars.target_ue_dl_mcs;
	  //((DCI1_10MHz_FDD_t *)&dlsch_pdu)->mcs              = (unsigned char) ((phy_vars_eNB->frame%1024)%28);      
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->ndi              = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->rv               = 0;
	  memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],(void *)&dlsch_pdu,sizeof(DCI1_10MHz_TDD_t));
	  
	  /*
	  //user2
	  DCI_pdu->dci_alloc[1].dci_length = sizeof_DCI1_10MHz_TDD_t; 
	  DCI_pdu->dci_alloc[1].L          = 2;
	  DCI_pdu->dci_alloc[1].rnti       = 0x1236;
	  DCI_pdu->dci_alloc[1].format     = format1;
	  DCI_pdu->dci_alloc[1].ra_flag    = 0;
	  
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->rballoc          = rballoc2;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->TPC              = 0;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->dai              = 0;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->harq_pid         = 1;
	  //((DCI1_10MHz_FDD_t *)&dlsch_pdu)->mcs              = (unsigned char) ((phy_vars_eNB->proc[subframe].frame%1024)%28);      
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->mcs              = openair_daq_vars.target_ue_dl_mcs;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->ndi              = 1;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->rv               = 0;
	  memcpy((void*)&DCI_pdu->dci_alloc[1].dci_pdu[0],(void *)&((DCI1_10MHz_FDD_t *)&dlsch_pdu)->,sizeof(DCI1_10MHz_TDD_t));
	  */
	}
	else {
	  DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1_10MHz_TDD_t; 
	
	  ((DCI1_10MHz_TDD_t *)&dlsch_pdu)->rballoc          = rballoc;
	  ((DCI1_10MHz_TDD_t *)&dlsch_pdu)->TPC              = 0;
	  ((DCI1_10MHz_TDD_t *)&dlsch_pdu)->dai              = 0;
	  ((DCI1_10MHz_TDD_t *)&dlsch_pdu)->harq_pid         = 0;
	  ((DCI1_10MHz_TDD_t *)&dlsch_pdu)->mcs              = openair_daq_vars.target_ue_dl_mcs;
	  //((DCI1_10MHz_TDD_t *)&dlsch_pdu)->mcs              = (unsigned char) ((phy_vars_eNB->frame%1024)%28);      
	  ((DCI1_10MHz_TDD_t *)&dlsch_pdu)->ndi              = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	  ((DCI1_10MHz_TDD_t *)&dlsch_pdu)->rv               = 0;
	  memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],(void *)&dlsch_pdu,sizeof(DCI1_10MHz_TDD_t));
	  
	  /*
	  //user2
	  DCI_pdu->dci_alloc[1].dci_length = sizeof_DCI1_10MHz_TDD_t; 
	  DCI_pdu->dci_alloc[1].L          = 2;
	  DCI_pdu->dci_alloc[1].rnti       = 0x1236;
	  DCI_pdu->dci_alloc[1].format     = format1;
	  DCI_pdu->dci_alloc[1].ra_flag    = 0;
	  
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->rballoc          = rballoc2;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->TPC              = 0;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->dai              = 0;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->harq_pid         = 1;
	  //((DCI1_10MHz_FDD_t *)&dlsch_pdu)->mcs              = (unsigned char) ((phy_vars_eNB->proc[subframe].frame%1024)%28);      
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->mcs              = openair_daq_vars.target_ue_dl_mcs;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->ndi              = 1;
	  ((DCI1_10MHz_FDD_t *)&dlsch_pdu)->rv               = 0;
	  memcpy((void*)&DCI_pdu->dci_alloc[1].dci_pdu[0],(void *)&((DCI1_10MHz_FDD_t *)&dlsch_pdu)->,sizeof(DCI1_10MHz_TDD_t));
	  */
	}

	break;
      case 100:
	if (phy_vars_eNB->lte_frame_parms.frame_type == FDD) {
	  DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1_5MHz_FDD_t; 
	
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->rballoc          = rballoc;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->TPC              = 0;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->harq_pid         = 0;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->mcs              = openair_daq_vars.target_ue_dl_mcs;
	  //((DCI1_5MHz_FDD_t *)&dlsch_pdu)->mcs              = (unsigned char) ((phy_vars_eNB->frame%1024)%28);      
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->ndi              = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->rv               = 0;
	  memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],(void *)&dlsch_pdu,sizeof(DCI1_5MHz_TDD_t));
	  
	  /*
	  //user2
	  DCI_pdu->dci_alloc[1].dci_length = sizeof_DCI1_5MHz_TDD_t; 
	  DCI_pdu->dci_alloc[1].L          = 2;
	  DCI_pdu->dci_alloc[1].rnti       = 0x1236;
	  DCI_pdu->dci_alloc[1].format     = format1;
	  DCI_pdu->dci_alloc[1].ra_flag    = 0;
	  
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->rballoc          = rballoc2;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->TPC              = 0;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->dai              = 0;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->harq_pid         = 1;
	  //((DCI1_5MHz_FDD_t *)&dlsch_pdu)->mcs              = (unsigned char) ((phy_vars_eNB->proc[subframe].frame%1024)%28);      
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->mcs              = openair_daq_vars.target_ue_dl_mcs;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->ndi              = 1;
	  ((DCI1_5MHz_FDD_t *)&dlsch_pdu)->rv               = 0;
	  memcpy((void*)&DCI_pdu->dci_alloc[1].dci_pdu[0],(void *)&((DCI1_5MHz_FDD_t *)&dlsch_pdu)->,sizeof(DCI1_5MHz_TDD_t));
	  */
	}
	else {
	  DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1_20MHz_TDD_t; 
	
	  ((DCI1_20MHz_TDD_t *)&dlsch_pdu)->rballoc          = rballoc;
	  ((DCI1_20MHz_TDD_t *)&dlsch_pdu)->TPC              = 0;
	  ((DCI1_20MHz_TDD_t *)&dlsch_pdu)->dai              = 0;
	  ((DCI1_20MHz_TDD_t *)&dlsch_pdu)->harq_pid         = 0;
	  ((DCI1_20MHz_TDD_t *)&dlsch_pdu)->mcs              = openair_daq_vars.target_ue_dl_mcs;
	  //((DCI1_20MHz_TDD_t *)&dlsch_pdu)->mcs              = (unsigned char) ((phy_vars_eNB->frame%1024)%28);      
	  ((DCI1_20MHz_TDD_t *)&dlsch_pdu)->ndi              = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
	  ((DCI1_20MHz_TDD_t *)&dlsch_pdu)->rv               = 0;
	  memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],(void *)&dlsch_pdu,sizeof(DCI1_20MHz_TDD_t));
	  
	  /*
	  //user2
	  DCI_pdu->dci_alloc[1].dci_length = sizeof_DCI1_20MHz_TDD_t; 
	  DCI_pdu->dci_alloc[1].L          = 2;
	  DCI_pdu->dci_alloc[1].rnti       = 0x1236;
	  DCI_pdu->dci_alloc[1].format     = format1;
	  DCI_pdu->dci_alloc[1].ra_flag    = 0;
	  
	  ((DCI1_20MHz_FDD_t *)&dlsch_pdu)->rballoc          = rballoc2;
	  ((DCI1_20MHz_FDD_t *)&dlsch_pdu)->TPC              = 0;
	  ((DCI1_20MHz_FDD_t *)&dlsch_pdu)->dai              = 0;
	  ((DCI1_20MHz_FDD_t *)&dlsch_pdu)->harq_pid         = 1;
	  //((DCI1_20MHz_FDD_t *)&dlsch_pdu)->mcs              = (unsigned char) ((phy_vars_eNB->proc[subframe].frame%1024)%28);      
	  ((DCI1_20MHz_FDD_t *)&dlsch_pdu)->mcs              = openair_daq_vars.target_ue_dl_mcs;
	  ((DCI1_20MHz_FDD_t *)&dlsch_pdu)->ndi              = 1;
	  ((DCI1_20MHz_FDD_t *)&dlsch_pdu)->rv               = 0;
	  memcpy((void*)&DCI_pdu->dci_alloc[1].dci_pdu[0],(void *)&((DCI1_20MHz_FDD_t *)&dlsch_pdu)->,sizeof(DCI1_5MHz_TDD_t));
	  */
	}
	break;
      }
      
    }
    else if (transmission_mode==5) {
      DCI_pdu->Num_ue_spec_dci = 2;
      // user 1
      DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1E_5MHz_2A_M10PRB_TDD_t; 
      DCI_pdu->dci_alloc[0].L          = 3;
      DCI_pdu->dci_alloc[0].rnti       = 0x1235;
      DCI_pdu->dci_alloc[0].format     = format1E_2A_M10PRB;
      DCI_pdu->dci_alloc[0].ra_flag    = 0;
      
      DLSCH_alloc_pdu1E.tpmi             = 5; //5=use feedback
      DLSCH_alloc_pdu1E.rv               = 0;
      DLSCH_alloc_pdu1E.ndi              = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
      //DLSCH_alloc_pdu1E.mcs            = cqi_to_mcs[phy_vars_eNB->eNB_UE_stats->DL_cqi[0]];
      //DLSCH_alloc_pdu1E.mcs            = (unsigned char) (taus()%28);
      DLSCH_alloc_pdu1E.mcs              = openair_daq_vars.target_ue_dl_mcs;
      //DLSCH_alloc_pdu1E.mcs            = (unsigned char) ((phy_vars_eNB->proc[subframe].frame%1024)%28);      
      phy_vars_eNB->eNB_UE_stats[0].dlsch_mcs1 = DLSCH_alloc_pdu1E.mcs;
      DLSCH_alloc_pdu1E.harq_pid         = 0;
      DLSCH_alloc_pdu1E.dai              = 0;
      DLSCH_alloc_pdu1E.TPC              = 0;
      DLSCH_alloc_pdu1E.rballoc          = openair_daq_vars.ue_dl_rb_alloc;
      DLSCH_alloc_pdu1E.rah              = 0;
      DLSCH_alloc_pdu1E.dl_power_off     = 0; //0=second user present
      memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],(void *)&DLSCH_alloc_pdu1E,sizeof(DCI1E_5MHz_2A_M10PRB_TDD_t));
      
      //user 2
      DCI_pdu->dci_alloc[1].dci_length = sizeof_DCI1E_5MHz_2A_M10PRB_TDD_t; 
      DCI_pdu->dci_alloc[1].L          = 0;
      DCI_pdu->dci_alloc[1].rnti       = 0x1236;
      DCI_pdu->dci_alloc[1].format     = format1E_2A_M10PRB;
      DCI_pdu->dci_alloc[1].ra_flag    = 0;
      //DLSCH_alloc_pdu1E.mcs            = openair_daq_vars.target_ue_dl_mcs; 
      //DLSCH_alloc_pdu1E.mcs            = (unsigned char) (taus()%28);
      //DLSCH_alloc_pdu1E.mcs            = (unsigned char) ((phy_vars_eNB->frame%1024)%28);
      DLSCH_alloc_pdu1E.mcs            = (unsigned char) (((phy_vars_eNB->proc[sched_subframe].frame_tx%1024)/3)%28);
      phy_vars_eNB->eNB_UE_stats[1].dlsch_mcs1 = DLSCH_alloc_pdu1E.mcs;
      
      memcpy((void*)&DCI_pdu->dci_alloc[1].dci_pdu[0],(void *)&DLSCH_alloc_pdu1E,sizeof(DCI1E_5MHz_2A_M10PRB_TDD_t));

      // set the precoder of the second UE orthogonal to the first
      phy_vars_eNB->eNB_UE_stats[1].DL_pmi_single = (phy_vars_eNB->eNB_UE_stats[0].DL_pmi_single ^ 0x1555); 
    }
    break;
    /*
      case 8:
      DCI_pdu->Num_common_dci = 1;
      DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1A_5MHz_TDD_1_6_t;
      DCI_pdu->dci_alloc[0].L          = 2;
      DCI_pdu->dci_alloc[0].rnti       = 0xbeef;
      DCI_pdu->dci_alloc[0].format     = format1A;
      DCI_pdu->dci_alloc[0].ra_flag    = 1;

      RA_alloc_pdu.type                = 1;
      RA_alloc_pdu.vrb_type            = 0;
      RA_alloc_pdu.rballoc             = computeRIV(25,12,3);
      RA_alloc_pdu.ndi      = 1;
      RA_alloc_pdu.rv       = 1;
      RA_alloc_pdu.mcs      = 4;
      RA_alloc_pdu.harq_pid = 0;
      RA_alloc_pdu.TPC      = 1;

      memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],&RA_alloc_pdu,sizeof(DCI1A_5MHz_TDD_1_6_t));
      break;
    */
  case 9:
    DCI_pdu->Num_ue_spec_dci = 1;

    //user 1
    if (phy_vars_eNB->lte_frame_parms.frame_type == FDD)
      DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI0_5MHz_FDD_t ; 
    else
      DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI0_5MHz_TDD_1_6_t ; 
    DCI_pdu->dci_alloc[0].L          = 2;
    DCI_pdu->dci_alloc[0].rnti       = 0x1235;
    DCI_pdu->dci_alloc[0].format     = format0;
    DCI_pdu->dci_alloc[0].ra_flag    = 0;

    UL_alloc_pdu.type    = 0;
    UL_alloc_pdu.hopping = 0;
    UL_alloc_pdu.rballoc = computeRIV(25,2,openair_daq_vars.ue_ul_nb_rb);
    UL_alloc_pdu.mcs     = openair_daq_vars.target_ue_ul_mcs;
    UL_alloc_pdu.ndi     = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
    UL_alloc_pdu.TPC     = 0;
    UL_alloc_pdu.cshift  = 0;
    UL_alloc_pdu.dai     = 0;
    UL_alloc_pdu.cqi_req = 1;
    memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],(void *)&UL_alloc_pdu,sizeof(DCI0_5MHz_TDD_1_6_t));
       
    // user 2
    /*
    DCI_pdu->dci_alloc[1].dci_length = sizeof_DCI0_5MHz_TDD_1_6_t ; 
    DCI_pdu->dci_alloc[1].L          = 2;
    DCI_pdu->dci_alloc[1].rnti       = 0x1236;
    DCI_pdu->dci_alloc[1].format     = format0;
    DCI_pdu->dci_alloc[1].ra_flag    = 0;

    UL_alloc_pdu.type    = 0;
    UL_alloc_pdu.hopping = 0;
    if (cooperation_flag==0)
      UL_alloc_pdu.rballoc = computeRIV(25,2+openair_daq_vars.ue_ul_nb_rb,openair_daq_vars.ue_ul_nb_rb);
    else 
      UL_alloc_pdu.rballoc = computeRIV(25,0,openair_daq_vars.ue_ul_nb_rb);
    UL_alloc_pdu.mcs     = openair_daq_vars.target_ue_ul_mcs;
    UL_alloc_pdu.ndi     = phy_vars_eNB->proc[sched_subframe].frame_tx&1;
    UL_alloc_pdu.TPC     = 0;
    if ((cooperation_flag==0) || (cooperation_flag==1))
      UL_alloc_pdu.cshift  = 0;
    else
      UL_alloc_pdu.cshift  = 1;
    UL_alloc_pdu.dai     = 0;
    UL_alloc_pdu.cqi_req = 1;
    memcpy((void*)&DCI_pdu->dci_alloc[1].dci_pdu[0],(void *)&UL_alloc_pdu,sizeof(DCI0_5MHz_TDD_1_6_t));
    */
    break;

  default:
    break;
  }

  DCI_pdu->nCCE = 0;
  for (i=0;i<DCI_pdu->Num_common_dci+DCI_pdu->Num_ue_spec_dci;i++) {
    DCI_pdu->nCCE += (1<<(DCI_pdu->dci_alloc[i].L));
  }

}

#ifdef EMOS
void fill_dci_emos(DCI_PDU *DCI_pdu, uint8_t subframe, PHY_VARS_eNB *phy_vars_eNB) {

  int i;
  uint8_t cooperation_flag = phy_vars_eNB->cooperation_flag;
  uint8_t transmission_mode = phy_vars_eNB->transmission_mode[0];

  //uint32_t rballoc = 0x00F0;
  //uint32_t rballoc2 = 0x000F;
  /*
    uint32_t rand = taus();
    if ((subframe==8) || (subframe==9) || (subframe==0))
    rand = (rand%5)+5;
    else
    rand = (rand%4)+5;
  */

  DCI_pdu->Num_common_dci = 0;
  DCI_pdu->Num_ue_spec_dci=0;

  switch (subframe) {
  case 5:
    DCI_pdu->Num_ue_spec_dci = 1;
    
    if (transmission_mode<3) {
      //user 1
      DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1_5MHz_TDD_t; 
      DCI_pdu->dci_alloc[0].L          = 2;
      DCI_pdu->dci_alloc[0].rnti       = 0x1235;
      DCI_pdu->dci_alloc[0].format     = format1;
      DCI_pdu->dci_alloc[0].ra_flag    = 0;
      
      DLSCH_alloc_pdu.rballoc          = openair_daq_vars.ue_dl_rb_alloc;
      DLSCH_alloc_pdu.TPC              = 0;
      DLSCH_alloc_pdu.dai              = 0;
      DLSCH_alloc_pdu.harq_pid         = 1;
      DLSCH_alloc_pdu.mcs              = openair_daq_vars.target_ue_dl_mcs;
      DLSCH_alloc_pdu.ndi              = 1;
      DLSCH_alloc_pdu.rv               = 0;
      memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],(void *)&DLSCH_alloc_pdu,sizeof(DCI1_5MHz_TDD_t));

      /*
      //user2
      DCI_pdu->dci_alloc[1].dci_length = sizeof_DCI1_5MHz_TDD_t; 
      DCI_pdu->dci_alloc[1].L          = 2;
      DCI_pdu->dci_alloc[1].rnti       = 0x1236;
      DCI_pdu->dci_alloc[1].format     = format1;
      DCI_pdu->dci_alloc[1].ra_flag    = 0;
      
      DLSCH_alloc_pdu.rballoc          = rballoc2;
      DLSCH_alloc_pdu.TPC              = 0;
      DLSCH_alloc_pdu.dai              = 0;
      DLSCH_alloc_pdu.harq_pid         = 1;
      DLSCH_alloc_pdu.mcs              = openair_daq_vars.target_ue_dl_mcs;
      DLSCH_alloc_pdu.ndi              = 1;
      DLSCH_alloc_pdu.rv               = 0;
      memcpy((void*)&DCI_pdu->dci_alloc[1].dci_pdu[0],(void *)&DLSCH_alloc_pdu,sizeof(DCI1_5MHz_TDD_t));
      */
    }
    else if (transmission_mode==5) {
      DCI_pdu->Num_ue_spec_dci = 2;
      // user 1
      DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1E_5MHz_2A_M10PRB_TDD_t; 
      DCI_pdu->dci_alloc[0].L          = 2;
      DCI_pdu->dci_alloc[0].rnti       = 0x1235;
      DCI_pdu->dci_alloc[0].format     = format1E_2A_M10PRB;
      DCI_pdu->dci_alloc[0].ra_flag    = 0;
      
      DLSCH_alloc_pdu1E.tpmi             = 5; //5=use feedback
      DLSCH_alloc_pdu1E.rv               = 0;
      DLSCH_alloc_pdu1E.ndi              = 1;
      DLSCH_alloc_pdu1E.mcs              = openair_daq_vars.target_ue_dl_mcs;
      DLSCH_alloc_pdu1E.harq_pid         = 1;
      DLSCH_alloc_pdu1E.dai              = 0;
      DLSCH_alloc_pdu1E.TPC              = 0;
      DLSCH_alloc_pdu1E.rballoc          = openair_daq_vars.ue_dl_rb_alloc;
      DLSCH_alloc_pdu1E.rah              = 0;
      DLSCH_alloc_pdu1E.dl_power_off     = 0; //0=second user present
      memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],(void *)&DLSCH_alloc_pdu1E,sizeof(DCI1E_5MHz_2A_M10PRB_TDD_t));
      
      //user 2
      DCI_pdu->dci_alloc[1].dci_length = sizeof_DCI1E_5MHz_2A_M10PRB_TDD_t; 
      DCI_pdu->dci_alloc[1].L          = 2;
      DCI_pdu->dci_alloc[1].rnti       = 0x1236;
      DCI_pdu->dci_alloc[1].format     = format1E_2A_M10PRB;
      DCI_pdu->dci_alloc[1].ra_flag    = 0;
      
      memcpy((void*)&DCI_pdu->dci_alloc[1].dci_pdu[0],(void *)&DLSCH_alloc_pdu1E,sizeof(DCI1E_5MHz_2A_M10PRB_TDD_t));

      // set the precoder of the second UE orthogonal to the first
      phy_vars_eNB->eNB_UE_stats[1].DL_pmi_single = (phy_vars_eNB->eNB_UE_stats[0].DL_pmi_single ^ 0x1555); 
    }
    break;

  case 7:
    DCI_pdu->Num_common_dci = 1;
    DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI1A_5MHz_TDD_1_6_t;
    DCI_pdu->dci_alloc[0].L          = 2;
    DCI_pdu->dci_alloc[0].rnti       = 0xbeef;
    DCI_pdu->dci_alloc[0].format     = format1A;
    DCI_pdu->dci_alloc[0].ra_flag    = 1;

    RA_alloc_pdu.type                = 1;
    RA_alloc_pdu.vrb_type            = 0;
    RA_alloc_pdu.rballoc             = computeRIV(25,12,3);
    RA_alloc_pdu.ndi      = 1;
    RA_alloc_pdu.rv       = 1;
    RA_alloc_pdu.mcs      = 4;
    RA_alloc_pdu.harq_pid = 0;
    RA_alloc_pdu.TPC      = 1;

    memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],&RA_alloc_pdu,sizeof(DCI1A_5MHz_TDD_1_6_t));
    break;

  case 9:
    DCI_pdu->Num_ue_spec_dci = 1;

    //user 1
    DCI_pdu->dci_alloc[0].dci_length = sizeof_DCI0_5MHz_TDD_1_6_t ; 
    DCI_pdu->dci_alloc[0].L          = 2;
    DCI_pdu->dci_alloc[0].rnti       = 0x1235;
    DCI_pdu->dci_alloc[0].format     = format0;
    DCI_pdu->dci_alloc[0].ra_flag    = 0;

    UL_alloc_pdu.type    = 0;
    UL_alloc_pdu.hopping = 0;
    UL_alloc_pdu.rballoc = computeRIV(25,0,openair_daq_vars.ue_ul_nb_rb);
    UL_alloc_pdu.mcs     = openair_daq_vars.target_ue_ul_mcs;
    UL_alloc_pdu.ndi     = 1;
    UL_alloc_pdu.TPC     = 0;
    UL_alloc_pdu.cshift  = 0;
    UL_alloc_pdu.dai     = 0;
    UL_alloc_pdu.cqi_req = 1;
    memcpy((void*)&DCI_pdu->dci_alloc[0].dci_pdu[0],(void *)&UL_alloc_pdu,sizeof(DCI0_5MHz_TDD_1_6_t));

    /*
    //user 2
    DCI_pdu->dci_alloc[1].dci_length = sizeof_DCI0_5MHz_TDD_1_6_t ; 
    DCI_pdu->dci_alloc[1].L          = 2;
    DCI_pdu->dci_alloc[1].rnti       = 0x1236;
    DCI_pdu->dci_alloc[1].format     = format0;
    DCI_pdu->dci_alloc[1].ra_flag    = 0;

    UL_alloc_pdu.type    = 0;
    UL_alloc_pdu.hopping = 0;
    if (cooperation_flag==0)
    UL_alloc_pdu.rballoc = computeRIV(25,2+openair_daq_vars.ue_ul_nb_rb,openair_daq_vars.ue_ul_nb_rb);
    else 
    UL_alloc_pdu.rballoc = computeRIV(25,0,openair_daq_vars.ue_ul_nb_rb);
    UL_alloc_pdu.mcs     = openair_daq_vars.target_ue_ul_mcs;
    UL_alloc_pdu.ndi     = 1;
    UL_alloc_pdu.TPC     = 0;
    if ((cooperation_flag==0) || (cooperation_flag==1))
    UL_alloc_pdu.cshift  = 0;
    else
    UL_alloc_pdu.cshift  = 1;
    UL_alloc_pdu.dai     = 0;
    UL_alloc_pdu.cqi_req = 1;
    memcpy((void*)&DCI_pdu->dci_alloc[1].dci_pdu[0],(void *)&UL_alloc_pdu,sizeof(DCI0_5MHz_TDD_1_6_t));
    */
    break;
    
  default:
    break;
  }

  DCI_pdu->nCCE = 0;
  for (i=0;i<DCI_pdu->Num_common_dci+DCI_pdu->Num_ue_spec_dci;i++) {
    DCI_pdu->nCCE += (1<<(DCI_pdu->dci_alloc[i].L));
  }

}
#endif //EMOS
#endif //OPENAIR2

#define AMP_OVER_SQRT2 ((AMP*ONE_OVER_SQRT2_Q15)>>15)
#define AMP_OVER_2 (AMP>>1)
int QPSK[4]={AMP_OVER_SQRT2|(AMP_OVER_SQRT2<<16),AMP_OVER_SQRT2|((65536-AMP_OVER_SQRT2)<<16),((65536-AMP_OVER_SQRT2)<<16)|AMP_OVER_SQRT2,((65536-AMP_OVER_SQRT2)<<16)|(65536-AMP_OVER_SQRT2)};
int QPSK2[4]={AMP_OVER_2|(AMP_OVER_2<<16),AMP_OVER_2|((65536-AMP_OVER_2)<<16),((65536-AMP_OVER_2)<<16)|AMP_OVER_2,((65536-AMP_OVER_2)<<16)|(65536-AMP_OVER_2)};


#if defined(ENABLE_ITTI)
#   if defined(ENABLE_RAL)
extern PHY_MEASUREMENTS PHY_measurements;

void phy_eNB_lte_measurement_thresholds_test_and_report(instance_t instanceP, ral_threshold_phy_t* threshold_phy_pP, uint16_t valP) {
  MessageDef *message_p = NULL;
  if (
      (
       ((threshold_phy_pP->threshold.threshold_val <  valP) && (threshold_phy_pP->threshold.threshold_xdir == RAL_ABOVE_THRESHOLD)) ||
       ((threshold_phy_pP->threshold.threshold_val >  valP) && (threshold_phy_pP->threshold.threshold_xdir == RAL_BELOW_THRESHOLD))
       )  ||
      (threshold_phy_pP->threshold.threshold_xdir == RAL_NO_THRESHOLD)
      ){
    message_p = itti_alloc_new_message(TASK_PHY_ENB , PHY_MEAS_REPORT_IND);
    memset(&PHY_MEAS_REPORT_IND(message_p), 0, sizeof(PHY_MEAS_REPORT_IND(message_p)));

    memcpy(&PHY_MEAS_REPORT_IND (message_p).threshold,
	   &threshold_phy_pP->threshold,
	   sizeof(PHY_MEAS_REPORT_IND (message_p).threshold));

    memcpy(&PHY_MEAS_REPORT_IND (message_p).link_param,
	   &threshold_phy_pP->link_param,
	   sizeof(PHY_MEAS_REPORT_IND (message_p).link_param));\

    switch (threshold_phy_pP->link_param.choice) {
    case RAL_LINK_PARAM_CHOICE_LINK_PARAM_VAL:
      PHY_MEAS_REPORT_IND (message_p).link_param._union.link_param_val = valP;
      break;
    case RAL_LINK_PARAM_CHOICE_QOS_PARAM_VAL:
      //PHY_MEAS_REPORT_IND (message_p).link_param._union.qos_param_val.
      AssertFatal (1 == 0, "TO DO RAL_LINK_PARAM_CHOICE_QOS_PARAM_VAL\n");
      break;
    }
    itti_send_msg_to_task(TASK_RRC_ENB, instanceP, message_p);
  }
}

void phy_eNB_lte_check_measurement_thresholds(instance_t instanceP, ral_threshold_phy_t* threshold_phy_pP) {
  unsigned int  mod_id;

  mod_id = instanceP;

  switch (threshold_phy_pP->link_param.link_param_type.choice) {

  case RAL_LINK_PARAM_TYPE_CHOICE_GEN:
    switch (threshold_phy_pP->link_param.link_param_type._union.link_param_gen) {
    case RAL_LINK_PARAM_GEN_DATA_RATE:
      phy_eNB_lte_measurement_thresholds_test_and_report(instanceP, threshold_phy_pP, 0);
      break;
    case RAL_LINK_PARAM_GEN_SIGNAL_STRENGTH:
      phy_eNB_lte_measurement_thresholds_test_and_report(instanceP, threshold_phy_pP, 0);
      break;
    case RAL_LINK_PARAM_GEN_SINR:
      phy_eNB_lte_measurement_thresholds_test_and_report(instanceP, threshold_phy_pP, 0);
      break;
    case RAL_LINK_PARAM_GEN_THROUGHPUT:
      break;
    case RAL_LINK_PARAM_GEN_PACKET_ERROR_RATE:
      break;
    default:;
    }
    break;

  case RAL_LINK_PARAM_TYPE_CHOICE_LTE:
    switch (threshold_phy_pP->link_param.link_param_type._union.link_param_gen) {
    case RAL_LINK_PARAM_LTE_UE_RSRP:
      break;
    case RAL_LINK_PARAM_LTE_UE_RSRQ:
      break;
    case RAL_LINK_PARAM_LTE_UE_CQI:
      break;
    case RAL_LINK_PARAM_LTE_AVAILABLE_BW:
      break;
    case RAL_LINK_PARAM_LTE_PACKET_DELAY:
      break;
    case RAL_LINK_PARAM_LTE_PACKET_LOSS_RATE:
      break;
    case RAL_LINK_PARAM_LTE_L2_BUFFER_STATUS:
      break;
    case RAL_LINK_PARAM_LTE_MOBILE_NODE_CAPABILITIES:
      break;
    case RAL_LINK_PARAM_LTE_EMBMS_CAPABILITY:
      break;
    case RAL_LINK_PARAM_LTE_JUMBO_FEASIBILITY:
      break;
    case RAL_LINK_PARAM_LTE_JUMBO_SETUP_STATUS:
      break;
    case RAL_LINK_PARAM_LTE_NUM_ACTIVE_EMBMS_RECEIVERS_PER_FLOW:
      break;
    default:;
    }
    break;

  default:;
  }
}
#   endif
#endif



uint8 num_pdcch_symbols;                            // 2015/04/13  by CFL
uint8 pbch_pdu_tmp[3];                                  // 2015/04/13  by CFL
void phy_procedures_eNB_TX(unsigned char sched_subframe,PHY_VARS_eNB *phy_vars_eNB,uint8_t abstraction_flag,
                           relaying_type_t r_type,PHY_VARS_RN *phy_vars_rn) {

#ifdef MEASURETXTIME  
  RTIME time_start,time_end;
  time_start = rt_get_time_ns();
#endif
  UNUSED(phy_vars_rn);
  uint8_t *pbch_pdu=&phy_vars_eNB->pbch_pdu[0];
  uint16_t input_buffer_length, re_allocated=0;
  uint32_t i,ii,aa;
  uint8_t harq_pid;
  DCI_PDU *DCI_pdu;
  uint8_t *DLSCH_pdu=NULL,*DLSCH_pdu_1=NULL;
  DCI_PDU DCI_pdu_tmp;                                  //CFL 20150803
  
  DCI_ALLOC_t dci_alloc[8];                           // 2015/04/22 by CFL
  uint8_t DLSCH_pdu_tmp[768*8];                   // 2015/04/14 by CFL
  
  
  # define OAC 0                  				   // Open Acode PDSCH procedure
  # define OCF 1                     			   // Open III PDSCH procedure
  # define MEASTIME 0                         // Open MEASTIME
  # define IQ_TEST 0
  # define TEST_flag 1
  
#ifndef OPENAIR2
  DCI_PDU DCI_pdu_tmp;
  uint8_t DLSCH_pdu_tmp[768*8];
#endif
  int8_t UE_id;
  uint8_t ul_subframe;
  uint32_t ul_frame;
#ifdef Rel10
  MCH_PDU *mch_pduP;
  MCH_PDU  mch_pdu;
  uint8_t sync_area=255;
#endif

  int subframe = phy_vars_eNB->proc[sched_subframe].subframe_tx;     
  //int subframe = sched_subframe;                           // CFL 2015/04/10

  
  // ============ MIB =================  // CFL 2015/04/10
  phy_vars_eNB->lte_frame_parms.phich_config_common.phich_duration = 0 ;              // 0 = Normal, 1 = Extended
 //  phy_vars_eNB->lte_frame_parms.phich_config_common.phich_resource = one;           //  oneSixth = 1, half = 3, one = 6, two = 12       Note : <<PICOCHIP>>  PHICH_R_ONE_SIXTH = 0, PHICH_R_HALF = 1, PHICH_R_ONE = 2, PHICH_R_TWO = 3
  //LOG_I(PHY,"phich_resource = %d\n",phy_vars_eNB->lte_frame_parms.phich_config_common.phich_resource);
  
  start_meas(&phy_vars_eNB->phy_proc_tx);
  
#ifdef OPENAIR2
  // Get scheduling info for next subframe 
  //if (phy_vars_eNB->CC_id == 0)
    //mac_xface->eNB_dlsch_ulsch_scheduler(phy_vars_eNB->Mod_id,0,phy_vars_eNB->proc[sched_subframe].frame_tx,subframe);//,1); //20150915
#endif
  
  if (abstraction_flag==0) {
    // clear the transmit data array for the current subframe
    for (aa=0; aa<phy_vars_eNB->lte_frame_parms.nb_antennas_tx_eNB;aa++) {
      
      memset(&phy_vars_eNB->lte_eNB_common_vars.txdataF[0][aa][subframe*phy_vars_eNB->lte_frame_parms.ofdm_symbol_size*(phy_vars_eNB->lte_frame_parms.symbols_per_tti)],
	     0,phy_vars_eNB->lte_frame_parms.ofdm_symbol_size*(phy_vars_eNB->lte_frame_parms.symbols_per_tti)*sizeof(mod_sym_t));
    }
  }


 // ==================== Add ======================
    /* joe 20150331 get tx vector pointer*/   
   unsigned char rah, TPC, mcs, ndi, rv, aggregationLevel, cceStart,format,harqProc,vrbAssignFlag;
   uint16_t rnti;
   uint32 tbs, rballoc, coded_bits_per_codeword;
   tMacaTxVectorReq    *pTxVector       = NULL;
   tPhyDlschData       *pDlschDataPdu = NULL;    
   tMacaDciDlPduConfig *pDciPdu         = NULL;
   tMacaDlschPduConfig *pDlschPduConfig = NULL;    
   tMacaHiDciVectorReq    *pHiDciVectorReq   = NULL;
   tMacaHiPduConfig       *pHiPduConfig      = NULL;
   tMacaDciUlPduConfig    *pDciUlPduConfig   = NULL;
   
   
   pTxVector = &(sTxVectorReq[subframe]);
   pDlschDataPdu = &(sTxDataReq[subframe]);
   pHiDciVectorReq = &(sHiDciVectorReq[subframe]);
   //LOG_I(PHY,"txFrameNum = %d, txsubFrameNum = %d\n",pTxVector->txFrameNum,pTxVector->txSubframeNum);
   //LOG_I(PHY,"frame_tx = %d, subframetx = %d sched_subframe = %d\n",phy_vars_eNB->proc[sched_subframe].frame_tx,subframe,sched_subframe);

   
   
   /*
    if (phy_vars_eNB->lte_frame_parms.mode1_flag == 1)
		LOG_I(PHY,"Generating PBCH for SISO \n");
	else
		LOG_I(PHY,"Generating PBCH for Transmit diversity \n");

   */
 // ================ START TX  PROCESS======================
    
   if (abstraction_flag==0){
		
#if MEASTIME		
	// Meas. Time	
		clock_gettime(CLOCK_REALTIME, &requestStart);   // Time start*/
#endif

// ================== Generate RS ===========================
      generate_pilots_slot(phy_vars_eNB,
			   phy_vars_eNB->lte_eNB_common_vars.txdataF[0],
			   AMP,
			   subframe<<1,0);
      generate_pilots_slot(phy_vars_eNB,
			   phy_vars_eNB->lte_eNB_common_vars.txdataF[0],
			   AMP,
			   (subframe<<1)+1,0);
	
// ================= Generate Sync signal ======================	
    // First half of PSS/SSS (FDD)
	
    if (subframe == 0) {
		if (phy_vars_eNB->lte_frame_parms.frame_type == FDD) {
			generate_pss(phy_vars_eNB->lte_eNB_common_vars.txdataF[0],
								AMP_BPSK,
								&phy_vars_eNB->lte_frame_parms,
								(phy_vars_eNB->lte_frame_parms.Ncp==NORMAL) ? 6 : 5,
								0);
			generate_sss(phy_vars_eNB->lte_eNB_common_vars.txdataF[0],
								AMP_BPSK,
								&phy_vars_eNB->lte_frame_parms,
								(phy_vars_eNB->lte_frame_parms.Ncp==NORMAL) ? 5 : 4,
								0);
	    
		}
      }    
   }
 
// ================== Reach BCH PDU from FAPI================
// 2015/04/13  by CFL

  if ((subframe == 0) && (((pTxVector->txFrameNum)%4) == 0)){
	  pbch_pdu_tmp[2] = pTxVector->bchConfig.bchPdu[2];
      pbch_pdu_tmp[1] = pTxVector->bchConfig.bchPdu[1];
      pbch_pdu_tmp[0] = pTxVector->bchConfig.bchPdu[0];
  }
 
// ================== Generate PBCH =====================

  if (subframe == 0) {


	  pbch_pdu[2] =  pbch_pdu_tmp[0] ;
      pbch_pdu[1] =  pbch_pdu_tmp[1] ;
      pbch_pdu[0] =  pbch_pdu_tmp[2] ;


    if (abstraction_flag==0) {
	
      generate_pbch(&phy_vars_eNB->lte_eNB_pbch,
		    phy_vars_eNB->lte_eNB_common_vars.txdataF[0],
		    AMP,
		    &phy_vars_eNB->lte_frame_parms,
		    pbch_pdu,
			pTxVector->txFrameNum&3);
			//phy_vars_eNB->proc[sched_subframe].frame_tx&3);
    }
  }
  
// ================ Generate Sync signal ======================	
  // Second half of PSS/SSS (FDD)
  if (subframe == 5) {
      
    if (abstraction_flag==0) {
	
      if (phy_vars_eNB->lte_frame_parms.frame_type == FDD) {
	generate_pss(phy_vars_eNB->lte_eNB_common_vars.txdataF[0],
		     AMP_BPSK,
		     &phy_vars_eNB->lte_frame_parms,
                     (phy_vars_eNB->lte_frame_parms.Ncp==NORMAL) ? 6 : 5,
		     10);
	generate_sss(phy_vars_eNB->lte_eNB_common_vars.txdataF[0],
		     AMP_BPSK,
		     &phy_vars_eNB->lte_frame_parms,
                     (phy_vars_eNB->lte_frame_parms.Ncp==NORMAL) ? 5 : 4,
		     10);
	  
      }
    }
  }

#ifdef OPENAIR2
  DCI_pdu = &DCI_pdu_tmp;		
#else
  DCI_pdu = &DCI_pdu_tmp;
#endif


 // clear existing ulsch dci allocations before applying info from MAC  (this is table
  ul_subframe = pdcch_alloc2ul_subframe(&phy_vars_eNB->lte_frame_parms,subframe);
  ul_frame = pdcch_alloc2ul_frame(&phy_vars_eNB->lte_frame_parms,phy_vars_eNB->proc[sched_subframe].frame_tx,subframe);
  
  if ((subframe_select(&phy_vars_eNB->lte_frame_parms,ul_subframe)==SF_UL) ||
      (phy_vars_eNB->lte_frame_parms.frame_type == FDD)) {
    harq_pid = subframe2harq_pid(&phy_vars_eNB->lte_frame_parms,ul_frame,ul_subframe);
	harq_pid = 0;
    for (i=0;i<NUMBER_OF_UE_MAX;i++)
      if (phy_vars_eNB->ulsch_eNB[i]) {
	phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->dci_alloc=0;
	phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->rar_alloc=0;
      }
  }
  
  // clear previous allocation information for all UEs
  for (i=0;i<NUMBER_OF_UE_MAX;i++) {
    phy_vars_eNB->dlsch_eNB[i][0]->subframe_tx[subframe] = 0;
  }


// ===================================================
// =================== Try to fill DCI ======================
// =================================================== 
// Parameter Setting                                                 By CF 2015/04/01
// ============ Get CFI and CCE index from FAPI==============
  num_pdcch_symbols = pTxVector->cfiConfig.cfi;                               // CFI
  DCI_pdu->nCCE = cceStart;                                                   // CCE index
if (num_pdcch_symbols==0){
	num_pdcch_symbols = 2;
	  LOG_I(PHY,"num_pdcch_symbols = %d in subframe = %d\n", num_pdcch_symbols,subframe);
}  
  
  DCI_pdu->Num_common_dci = 0;
  DCI_pdu->Num_ue_spec_dci = 0;
  
  
  # if TEST_flag
    // =========== For Test MIMO ==============
	 if ((subframe ==1)||(subframe ==2)||(subframe ==3)||(subframe ==4)){

	    pTxVector->DciNum =  1;
        pDciPdu = &(pTxVector->dciDlConfig.dciDlPdu[0]);              // 0 repress DCI no.0
		pDciPdu->format = 0;                                 // Acode format +1 = picochip format
		pDciPdu->rnti = 12;
        pDciPdu->resAllocType = 0;
	    pDciPdu->tpc = 0;
	    pDciPdu->harqProc = 0;
  	    pDciPdu->mcs_tb1 =28;
		pDciPdu->mcs_tb2 = 20;
	    pDciPdu->ndi_tb1 = 0;
	    pDciPdu->irv_tb1 = 0;					
		pDciPdu->cceStart = 4;
		pDciPdu->aggregationLevel = 2;                   // *********** must to be log ***************
		pDciPdu->vrbAssignFlag = 0;
	    pDciPdu->resAlloc[0] = 0;                     // FAPI to Acode =>4byte to oct
        pDciPdu->resAlloc[1] = 0;   
        pDciPdu->resAlloc[2] = 0;   
        pDciPdu->resAlloc[3] = 0;   
		pDciPdu->scrambleType = 1;
		pDciPdu->allocPrachFlag = 0;
	    pDciPdu->prachMaskIndex = 1;
	    pDciPdu->preambleIndex = 52;
        pDciPdu->swapFlag = 0;

        pDciPdu->ndi_tb2 = 0;
        pDciPdu->irv_tb2 = 0;	
        pDciPdu->tpmi = 0;	                      // 0(36.212) to 1(36.211), 1(36.212) to 2(36.211)
		pDciPdu->precodingInfo = 1;	
	 }
	 
#endif
  
  
  // ==========Set DCI value ================
  // Catch DCI info.
  
if (pTxVector->DciNum==0){
	    pDciPdu = &(pTxVector->dciDlConfig.dciDlPdu[0]);              // 0 repress DCI no.0

		format = pDciPdu->format+1;                                 // Acode format +1 = picochip format
		rnti = pDciPdu->rnti;
        rah =  pDciPdu->resAllocType;
	    TPC =  pDciPdu->tpc;
	    harqProc =  pDciPdu->harqProc;
  	    mcs =  pDciPdu->mcs_tb1;
	    ndi =  pDciPdu->ndi_tb1;
	    rv =  pDciPdu->irv_tb1;					
		cceStart = pDciPdu->cceStart;
		aggregationLevel = log(pDciPdu->aggregationLevel)/log(2);                   // *********** must to be log ***************
		vrbAssignFlag = pDciPdu->vrbAssignFlag;
	    rballoc = ((pDciPdu->resAlloc[0]<<24)+(pDciPdu->resAlloc[1]<<16)+(pDciPdu->resAlloc[2]<<8)+(pDciPdu->resAlloc[3]));                     // FAPI to Acode =>4byte to oct
		
}

  // =======================================
uint32_t DLSCH_RB_ALLOC;
uint8_t DLSCH_alloc_pdu[8];                                                 // temp DCI location
uint8_t ULSCH_alloc_pdu[8];
int dci_length_bytes=0,dci_length=0,numCCE = 0,RA_rnti = 0;

for(i=0 ; i<pTxVector->DciNum ; i++){	
        pDciPdu = &(pTxVector->dciDlConfig.dciDlPdu[i]);              // 0 repress DCI no.0
				
		DCI_pdu->dci_alloc[i].format = pDciPdu->format+1;                                 // Acode format +1 = picochip format
		
		if (pDciPdu->format==5)                                                           // Mach FAPI DCI format to Acode
		  DCI_pdu->dci_alloc[i].format =format2;
		if (pDciPdu->format==6)
		  DCI_pdu->dci_alloc[i].format =format2A;

		aggregationLevel = log(pDciPdu->aggregationLevel)/log(2);                   // *********** must to be log ***************
		rnti = pDciPdu->rnti;
        rah =  pDciPdu->resAllocType;
	    TPC =  pDciPdu->tpc;
	    harqProc =  pDciPdu->harqProc;
  	    mcs =  pDciPdu->mcs_tb1;
	    ndi =  pDciPdu->ndi_tb1;
	    rv =  pDciPdu->irv_tb1;					
		cceStart = pDciPdu->cceStart;
		aggregationLevel = log(pDciPdu->aggregationLevel)/log(2);                   // *********** must to be log ***************
		vrbAssignFlag = pDciPdu->vrbAssignFlag;
	    rballoc = ((pDciPdu->resAlloc[0]<<24)+(pDciPdu->resAlloc[1]<<16)+(pDciPdu->resAlloc[2]<<8)+(pDciPdu->resAlloc[3]));                     // FAPI to Acode =>4byte to oct
		memset(&(DLSCH_alloc_pdu),0, sizeof(DLSCH_alloc_pdu)); //joe 20160427
// ============ DCI format 1  ================
if(DCI_pdu->dci_alloc[i].format == format1){

    //LOG_I(PHY,"Subframe %d use DCI Format : 1\n",subframe);	
    DCI_pdu->dci_alloc[i].L          = aggregationLevel;
    DCI_pdu->dci_alloc[i].rnti       = rnti;
    DCI_pdu->dci_alloc[i].ra_flag    = 0;
    switch (phy_vars_eNB->lte_frame_parms.N_RB_DL) {
    case 50:
        DCI_pdu->dci_alloc[i].dci_length = sizeof_DCI1_10MHz_FDD_t;
        DCI_pdu->dci_alloc[i].nCCE       = cceStart;
        
        dci_length = sizeof_DCI1_10MHz_FDD_t;
        dci_length_bytes = sizeof(DCI1_10MHz_FDD_t);
        
        ((DCI1_10MHz_FDD_t *)&DLSCH_alloc_pdu)->rah                = rah;
        ((DCI1_10MHz_FDD_t *)&DLSCH_alloc_pdu)->rballoc           = rballoc;
# if TEST_flag
    ((DCI1_10MHz_FDD_t *)&DLSCH_alloc_pdu)->rballoc           = 65535;
#endif
        ((DCI1_10MHz_FDD_t *)&DLSCH_alloc_pdu)->TPC              = TPC;
        ((DCI1_10MHz_FDD_t *)&DLSCH_alloc_pdu)->harq_pid        = harqProc;
        ((DCI1_10MHz_FDD_t *)&DLSCH_alloc_pdu)->mcs               = mcs;  
        ((DCI1_10MHz_FDD_t *)&DLSCH_alloc_pdu)->ndi                 = ndi;
        ((DCI1_10MHz_FDD_t *)&DLSCH_alloc_pdu)->rv                   = rv;
        

        memcpy((void*)&DCI_pdu->dci_alloc[i].dci_pdu[0],(void *)&DLSCH_alloc_pdu,sizeof(DCI1_10MHz_FDD_t));
    break;
    case 100:
        DCI_pdu->dci_alloc[i].dci_length = sizeof_DCI1_20MHz_FDD_t;
        DCI_pdu->dci_alloc[i].nCCE       = cceStart;
        
        dci_length = sizeof_DCI1_20MHz_FDD_t;
        dci_length_bytes = sizeof(DCI1_20MHz_FDD_t);
        
        ((DCI1_20MHz_FDD_t *)&DLSCH_alloc_pdu)->rah                = rah;
        ((DCI1_20MHz_FDD_t *)&DLSCH_alloc_pdu)->rballoc           = rballoc;
# if TEST_flag
    ((DCI1_20MHz_FDD_t *)&DLSCH_alloc_pdu)->rballoc           = 65535;
#endif
        ((DCI1_20MHz_FDD_t *)&DLSCH_alloc_pdu)->TPC              = TPC;
        ((DCI1_20MHz_FDD_t *)&DLSCH_alloc_pdu)->harq_pid        = harqProc;
        ((DCI1_20MHz_FDD_t *)&DLSCH_alloc_pdu)->mcs               = mcs;  
        ((DCI1_20MHz_FDD_t *)&DLSCH_alloc_pdu)->ndi                 = ndi;
        ((DCI1_20MHz_FDD_t *)&DLSCH_alloc_pdu)->rv                   = rv;
        

        memcpy((void*)&DCI_pdu->dci_alloc[i].dci_pdu[0],(void *)&DLSCH_alloc_pdu,sizeof(DCI1_20MHz_FDD_t));
    break;
    }
    DCI_pdu->Num_ue_spec_dci ++;
}

// ============ DCI format 1A  ================
else if(DCI_pdu->dci_alloc[i].format == format1A){
	
	if (pDciPdu->allocPrachFlag == 0){
        //LOG_I(PHY,"Subframe %d use DCI Format : 1A\n",subframe);	
        DCI_pdu->dci_alloc[i].L          = aggregationLevel;
        DCI_pdu->dci_alloc[i].rnti       = rnti;
        DCI_pdu->dci_alloc[i].ra_flag    = 0;
        switch (phy_vars_eNB->lte_frame_parms.N_RB_DL) {
        case 50:
            DCI_pdu->dci_alloc[i].dci_length = sizeof_DCI1A_10MHz_FDD_t;
            DCI_pdu->dci_alloc[i].nCCE       = cceStart;
                
            dci_length_bytes = sizeof(DCI1A_10MHz_FDD_t);

            ((DCI1A_10MHz_FDD_t*)&DLSCH_alloc_pdu)->type                          = 1;
            ((DCI1A_10MHz_FDD_t*)&DLSCH_alloc_pdu)->vrb_type                    = vrbAssignFlag;
            ((DCI1A_10MHz_FDD_t*)&DLSCH_alloc_pdu)->rballoc                      = rballoc;
# if TEST_flag
            ((DCI1_10MHz_FDD_t *)&DLSCH_alloc_pdu)->rballoc           = 103;
#endif
            ((DCI1A_10MHz_FDD_t*)&DLSCH_alloc_pdu)->ndi                           = ndi;
            ((DCI1A_10MHz_FDD_t*)&DLSCH_alloc_pdu)->rv                             = rv;
            ((DCI1A_10MHz_FDD_t*)&DLSCH_alloc_pdu)->mcs                         = mcs;
            ((DCI1A_10MHz_FDD_t*)&DLSCH_alloc_pdu)->harq_pid                   = harqProc;
            ((DCI1A_10MHz_FDD_t*)&DLSCH_alloc_pdu)->TPC                         = TPC;      
            
            memcpy((void*)&DCI_pdu->dci_alloc[i].dci_pdu[0],&DLSCH_alloc_pdu,sizeof(DCI1A_10MHz_FDD_t));	
        break;
        case 100:
            DCI_pdu->dci_alloc[i].dci_length = sizeof_DCI1A_20MHz_FDD_t;
            DCI_pdu->dci_alloc[i].nCCE       = cceStart;
                
            dci_length_bytes = sizeof(DCI1A_20MHz_FDD_t);

            ((DCI1A_20MHz_FDD_t*)&DLSCH_alloc_pdu)->type                          = 1;
            ((DCI1A_20MHz_FDD_t*)&DLSCH_alloc_pdu)->vrb_type                    = vrbAssignFlag;
            ((DCI1A_20MHz_FDD_t*)&DLSCH_alloc_pdu)->rballoc                      = rballoc;
# if TEST_flag
            ((DCI1A_20MHz_FDD_t *)&DLSCH_alloc_pdu)->rballoc           = 199;
#endif
            ((DCI1A_20MHz_FDD_t*)&DLSCH_alloc_pdu)->ndi                           = ndi;
            ((DCI1A_20MHz_FDD_t*)&DLSCH_alloc_pdu)->rv                             = rv;
            ((DCI1A_20MHz_FDD_t*)&DLSCH_alloc_pdu)->mcs                         = mcs;
            ((DCI1A_20MHz_FDD_t*)&DLSCH_alloc_pdu)->harq_pid                   = harqProc;
            ((DCI1A_20MHz_FDD_t*)&DLSCH_alloc_pdu)->TPC                         = TPC;      
            
            memcpy((void*)&DCI_pdu->dci_alloc[i].dci_pdu[0],&DLSCH_alloc_pdu,sizeof(DCI1A_20MHz_FDD_t));	
            break;
        }
        DCI_pdu->Num_common_dci ++;
	}
	
// Do Non-Contention RA                       20150918
	else if(pDciPdu->allocPrachFlag ==1){
     LOG_I(PHY,"Subframe %d use DCI Format : 1A For Non-contention RA\n",subframe);	
	/* DLSCH_RB_ALLOC = 0x7ff;    //0x1ffff;
	 DCI_pdu->dci_alloc[i].L          = aggregationLevel;
	 DCI_pdu->dci_alloc[i].rnti       = rnti;
     DCI_pdu->dci_alloc[i].ra_flag    = 0;
	 
    DCI_pdu->dci_alloc[i].dci_length = sizeof_DCI1A_RA_10MHz_FDD;
    DCI_pdu->dci_alloc[i].nCCE       = cceStart;
		
	dci_length_bytes = sizeof(DCI1A_RA_10MHz_FDD_t);

	((DCI1A_RA_10MHz_FDD_t*)&DLSCH_alloc_pdu)->type                               = 1;
	((DCI1A_RA_10MHz_FDD_t*)&DLSCH_alloc_pdu)->vrb_type                         = vrbAssignFlag;
	((DCI1A_RA_10MHz_FDD_t*)&DLSCH_alloc_pdu)->rballoc                            = DLSCH_RB_ALLOC;
	((DCI1A_RA_10MHz_FDD_t*)&DLSCH_alloc_pdu)->prach_mask_index          = pDciPdu->prachMaskIndex;
	((DCI1A_RA_10MHz_FDD_t*)&DLSCH_alloc_pdu)->preamble_index               = pDciPdu->preambleIndex;
   
	memcpy((void*)&DCI_pdu->dci_alloc[i].dci_pdu[0],&DLSCH_alloc_pdu,sizeof(DCI1A_RA_10MHz_FDD_t));	
    
	//LOG_I(PHY,"DLSCH_RB_ALLOC = %d , prachMaskIndex = %d, preambleIndex = %d\n",((DCI1A_RA_10MHz_FDD_t*)&DLSCH_alloc_pdu)->rballoc,((DCI1A_RA_10MHz_FDD_t*)&DLSCH_alloc_pdu)->prach_mask_index,((DCI1A_RA_10MHz_FDD_t*)&DLSCH_alloc_pdu)->preamble_index);
	
	DCI_pdu->Num_ue_spec_dci ++;
	
	*/
	}
	
  }
// ============ DCI format 2A  ================
  	else if(DCI_pdu->dci_alloc[i].format == format2A){
        //  LOG_I(PHY,"Subframe %d use DCI Format : 2A\n",subframe);	

        DCI_pdu->dci_alloc[i].L          = aggregationLevel;
        DCI_pdu->dci_alloc[i].rnti       = rnti;
        DCI_pdu->dci_alloc[i].ra_flag    = 0;
        switch (phy_vars_eNB->lte_frame_parms.N_RB_DL) {
        case 50:
            DCI_pdu->dci_alloc[i].dci_length = sizeof_DCI2A_10MHz_2A_FDD_t;
            DCI_pdu->dci_alloc[i].nCCE       = cceStart;
                
            dci_length_bytes = sizeof(DCI2A_10MHz_2A_FDD_t);
            
            ((DCI2A_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rah               = pDciPdu->resAllocType;
            ((DCI2A_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rballoc          = rballoc;
            ((DCI2A_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->TPC             = pDciPdu->tpc;
            ((DCI2A_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->harq_pid       = pDciPdu->harqProc;
            ((DCI2A_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->tb_swap       = pDciPdu->swapFlag;
            ((DCI2A_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->mcs1           = pDciPdu->mcs_tb1;
            ((DCI2A_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->ndi1             = pDciPdu->ndi_tb1;
            ((DCI2A_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rv1               = pDciPdu->irv_tb1;
            ((DCI2A_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->mcs2           = pDciPdu->mcs_tb2;
            ((DCI2A_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->ndi2             = pDciPdu->ndi_tb2;
            ((DCI2A_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rv2               = pDciPdu->irv_tb2;	
            
            memcpy((void*)&DCI_pdu->dci_alloc[i].dci_pdu[0],&DLSCH_alloc_pdu,sizeof(DCI2A_10MHz_2A_FDD_t));	
        break;
        case 100:
            DCI_pdu->dci_alloc[i].dci_length = sizeof_DCI2A_20MHz_2A_FDD_t;
            DCI_pdu->dci_alloc[i].nCCE       = cceStart;
                
            dci_length_bytes = sizeof(DCI2A_20MHz_2A_FDD_t);
            
            ((DCI2A_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rah               = pDciPdu->resAllocType;
            ((DCI2A_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rballoc          = rballoc;
            ((DCI2A_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->TPC             = pDciPdu->tpc;
            ((DCI2A_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->harq_pid       = pDciPdu->harqProc;
            ((DCI2A_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->tb_swap       = pDciPdu->swapFlag;
            ((DCI2A_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->mcs1           = pDciPdu->mcs_tb1;
            ((DCI2A_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->ndi1             = pDciPdu->ndi_tb1;
            ((DCI2A_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rv1               = pDciPdu->irv_tb1;
            ((DCI2A_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->mcs2           = pDciPdu->mcs_tb2;
            ((DCI2A_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->ndi2             = pDciPdu->ndi_tb2;
            ((DCI2A_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rv2               = pDciPdu->irv_tb2;	
            
            memcpy((void*)&DCI_pdu->dci_alloc[i].dci_pdu[0],&DLSCH_alloc_pdu,sizeof(DCI2A_20MHz_2A_FDD_t));	
        break;
        }
    DCI_pdu->Num_ue_spec_dci ++;
}
  
// ============ DCI format 2 ================
else if(DCI_pdu->dci_alloc[i].format == format2){
    LOG_I(PHY,"Subframe %d use DCI Format : 2\n",subframe);	

    DCI_pdu->dci_alloc[i].L          = aggregationLevel;
    DCI_pdu->dci_alloc[i].rnti       = rnti;
    DCI_pdu->dci_alloc[i].ra_flag    = 0;
    switch (phy_vars_eNB->lte_frame_parms.N_RB_DL) {
    case 50:
        DCI_pdu->dci_alloc[i].dci_length = sizeof_DCI2_10MHz_2A_FDD_t;
        DCI_pdu->dci_alloc[i].nCCE       = cceStart;
            
        dci_length_bytes = sizeof(DCI2_10MHz_2A_FDD_t);
        
        ((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rah               = pDciPdu->resAllocType;
        ((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rballoc          = rballoc;
        ((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->TPC             = pDciPdu->tpc;
        ((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->harq_pid       = pDciPdu->harqProc;
        ((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->tb_swap       = pDciPdu->swapFlag;
        ((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->mcs1           = pDciPdu->mcs_tb1;
        ((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->ndi1             = pDciPdu->ndi_tb1;
        ((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rv1               = pDciPdu->irv_tb1;
        ((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->mcs2           = pDciPdu->mcs_tb2;
        ((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->ndi2             = pDciPdu->ndi_tb2;
        ((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rv2               = pDciPdu->irv_tb2;	
        ((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->tpmi            = pDciPdu->precodingInfo;	
        //((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->tpmi            = pDciPdu->tpmi;	

        //LOG_I(PHY,precodingInfo = %d , tpmi = %d\n",pDciPdu->precodingInfo,pDciPdu->tpmi);	
       
 memcpy((void*)&DCI_pdu->dci_alloc[i].dci_pdu[0],&DLSCH_alloc_pdu,sizeof(DCI2_10MHz_2A_FDD_t));
	
    break;
    case 100:
        DCI_pdu->dci_alloc[i].dci_length = sizeof_DCI2_20MHz_2A_FDD_t;
        DCI_pdu->dci_alloc[i].nCCE       = cceStart;
            
        dci_length_bytes = sizeof(DCI2_20MHz_2A_FDD_t);
        
        ((DCI2_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rah               = pDciPdu->resAllocType;
        ((DCI2_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rballoc          = rballoc;
        ((DCI2_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->TPC             = pDciPdu->tpc;
        ((DCI2_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->harq_pid       = pDciPdu->harqProc;
        ((DCI2_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->tb_swap       = pDciPdu->swapFlag;
        ((DCI2_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->mcs1           = pDciPdu->mcs_tb1;
        ((DCI2_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->ndi1             = pDciPdu->ndi_tb1;
        ((DCI2_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rv1               = pDciPdu->irv_tb1;
        ((DCI2_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->mcs2           = pDciPdu->mcs_tb2;
        ((DCI2_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->ndi2             = pDciPdu->ndi_tb2;
        ((DCI2_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->rv2               = pDciPdu->irv_tb2;	
        ((DCI2_20MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->tpmi            = pDciPdu->precodingInfo;	
        //((DCI2_10MHz_2A_FDD_t *)&DLSCH_alloc_pdu)->tpmi            = pDciPdu->tpmi;	

        //LOG_I(PHY,precodingInfo = %d , tpmi = %d\n",pDciPdu->precodingInfo,pDciPdu->tpmi);	
        
        memcpy((void*)&DCI_pdu->dci_alloc[i].dci_pdu[0],&DLSCH_alloc_pdu,sizeof(DCI2_20MHz_2A_FDD_t));	
    break;
    }
    DCI_pdu->Num_ue_spec_dci ++;
}  
else
{
	LOG_E(PHY,"Unknown DCI format(%d) in copy dci\n",DCI_pdu->dci_alloc[i].format);
}

  init_nCCE_table();
  
/*
		LOG_I(PHY,"DciNum = %d\n",i+1);                     
		LOG_I(PHY,"subframe = %d\n",subframe);                       
		LOG_I(PHY,"format = %d\n",DCI_pdu->dci_alloc[i].format);                       // Acode format +1 = picochip format
	    LOG_I(PHY,"rnti = %d\n",pDciPdu->rnti);
	    LOG_I(PHY,"rah = %d\n",pDciPdu->resAllocType);
	    LOG_I(PHY,"rballoc = %d\n",((pDciPdu->resAlloc[0]<<24)+(pDciPdu->resAlloc[1]<<16)+(pDciPdu->resAlloc[2]<<8)+(pDciPdu->resAlloc[3])));
	    LOG_I(PHY,"TPC = %d\n",pDciPdu->tpc);
	    LOG_I(PHY,"harq_pid = %d\n",pDciPdu->harqProc);
        LOG_I(PHY,"mcs1 = %d\n",pDciPdu->mcs_tb1);
	    LOG_I(PHY,"ndi1 = %d\n",pDciPdu->ndi_tb1);
        LOG_I(PHY,"rv1= %d\n",pDciPdu->irv_tb1);
		LOG_I(PHY,"mcs2 = %d\n",pDciPdu->mcs_tb2);
	    LOG_I(PHY,"ndi2 = %d\n",pDciPdu->ndi_tb2);
        LOG_I(PHY,"rv2= %d\n",pDciPdu->irv_tb2);
		LOG_I(PHY,"nCCE = %d\n",pDciPdu->cceStart);
        LOG_I(PHY,"aggregationLevel = %d\n",aggregationLevel);   
		LOG_I(PHY,"vrbAssignFlag = %d\n",pDciPdu->vrbAssignFlag);
		LOG_I(PHY,"scrambleType = %d\n",pDciPdu->scrambleType);
		LOG_I(PHY,"resAlloc 0  = %d\nresAlloc 1  = %d\nresAlloc 2  = %d\nresAlloc 3  = %d\n\n",pDciPdu->resAlloc[0],pDciPdu->resAlloc[1],pDciPdu->resAlloc[2],pDciPdu->resAlloc[3]);  
		LOG_I(PHY,"allocPrachFlag = %d\n",pDciPdu->allocPrachFlag);   
		LOG_I(PHY,"prachMaskIndex = %d\n", pDciPdu->prachMaskIndex);   
		LOG_I(PHY,"preambleIndex = %d\n",pDciPdu->preambleIndex);   	
	    LOG_I(PHY,"precodingInfo = %d , tpmi = %d\n",pDciPdu->precodingInfo,pDciPdu->tpmi);	
  */
}  

 // ================================================================

// ==================== Pure PDSCH process ===========================
  # if OCF                                                              // Add by CFL 2015/04/16
     //get_TBS_DL
     unsigned char *input_buffer[2],*input_buffer_0[2];
	 UE_id = 0;
	 int k;
	 int dlinx = 0;                    //CFL_20150716

// =======================================================	 
for(i=0 ; i<pTxVector->DciNum ; i++){	              // loop for DCI number

	pDciPdu = &(pTxVector->dciDlConfig.dciDlPdu[i]);              // 0 repress DCI no.0       //CFL 2015/06/23
	
	if(pDciPdu->allocPrachFlag == 0){
	
    pDlschPduConfig = &(pTxVector->dlschConfig.dlschPdu[dlinx]);            // CFL2015/07/16
	phy_vars_eNB->pdsch_config_dedicated->p_a = pDlschPduConfig->pa ;       // CFL2015/0803
	
// ===================== Force Pa =============================

	   if (phy_vars_eNB->lte_frame_parms.mode1_flag == 0)
				phy_vars_eNB->pdsch_config_dedicated->p_a = 7 ;       // CFL2015/1006 force
	   if (phy_vars_eNB->lte_frame_parms.mode1_flag == 1)
				phy_vars_eNB->pdsch_config_dedicated->p_a = 4 ;       // CFL2015/1006 force	
			
			
		if (phy_vars_eNB->lte_frame_parms.mode1_flag == 0){
			if (phy_vars_eNB->pdsch_config_dedicated->p_a != 7)
	          LOG_I(PHY,"MIMO P_A = %d\n",phy_vars_eNB->pdsch_config_dedicated->p_a);
		}
		else if(phy_vars_eNB->lte_frame_parms.mode1_flag == 1){
			if (phy_vars_eNB->pdsch_config_dedicated->p_a != 4)
	          LOG_I(PHY,"SISO P_A = %d\n",phy_vars_eNB->pdsch_config_dedicated->p_a);
			}
	
	
	generate_eNB_dlsch_params_from_dci(subframe,                                          // subframe
					     &DCI_pdu->dci_alloc[i].dci_pdu[0],                                          // dci_pdu   Need fix :dci_alloc[i]    i = Dci number
					     DCI_pdu->dci_alloc[i].rnti,                                                      // C_rnti
					     DCI_pdu->dci_alloc[i].format,                                                  // DCI Format
					     phy_vars_eNB->dlsch_eNB[i],                                                // LTE_eNB_DLSCH_t
					     &phy_vars_eNB->lte_frame_parms,
					     phy_vars_eNB->pdsch_config_dedicated,
					     SI_RNTI,                                                                              // SI_rnti
					     pDciPdu->scrambleType,                                                                               // RA_rnti
						 //RA_rnti,                                                                               // RA_rnti
					     P_RNTI,                                                                               // P_rnti
					     phy_vars_eNB->eNB_UE_stats[0].DL_pmi_single);
						

  /* if ((phy_vars_eNB->dlsch_eNB[i][0]->harq_processes[pDciPdu->harqProc]->TBS/8)!=(pDlschPduConfig->DlschLength))
   {
	   printf("[Downlink]TBS ERROR frame(%d) subframe(%d) PHY TBS(%d) FPA TBS(%d)\n",phy_vars_eNB->proc[sched_subframe].frame_tx,phy_vars_eNB->proc[sched_subframe].subframe_tx,phy_vars_eNB->dlsch_eNB[i][0]->harq_processes[pDciPdu->harqProc]->TBS/8,pDlschPduConfig->DlschLength);
       printf("[PHY][Downlink] DCI Format = %d, rballoc = %d,mcs = %d rnti = %d,DciNum = %d,harq_pid = %d, scrambleType = %d,vrbAssignFlag = %d,nb_rb = %d, numberDCI = %d\n",DCI_pdu->dci_alloc[i].format,
                                           ((pDciPdu->resAlloc[0]<<24)+(pDciPdu->resAlloc[1]<<16)+(pDciPdu->resAlloc[2]<<8)+(pDciPdu->resAlloc[3])),
										   pDciPdu->mcs_tb1,
										   DCI_pdu->dci_alloc[i].rnti,
										   pTxVector->DciNum,
										   pDciPdu->harqProc,
										   pDciPdu->scrambleType,
										   pDciPdu->vrbAssignFlag,
										   phy_vars_eNB->dlsch_eNB[i][0]->harq_processes[pDciPdu->harqProc]->nb_rb,
										   i);
   
   }
*/
						   					   
    if ((DCI_pdu->dci_alloc[i].format != format0) &&(pTxVector->DciNum!=0)){          // mcs avoid to coredump
						 
		coded_bits_per_codeword = get_G(&phy_vars_eNB->lte_frame_parms,
									phy_vars_eNB->dlsch_eNB[i][0]->harq_processes[pDciPdu->harqProc]->nb_rb,
									phy_vars_eNB->dlsch_eNB[i][0]->harq_processes[pDciPdu->harqProc]->rb_alloc,
									get_Qm(phy_vars_eNB->dlsch_eNB[i][0]->harq_processes[pDciPdu->harqProc]->mcs),
									phy_vars_eNB->dlsch_eNB[i][0]->harq_processes[pDciPdu->harqProc]->Nl,
									num_pdcch_symbols,
									phy_vars_eNB->proc[sched_subframe].frame_tx,
									subframe);

     }

	 
	 # if TEST_flag
// =============Test 2 dci data==================	  
	k=1;
if((DCI_pdu->dci_alloc[i].format==format2)  || (DCI_pdu->dci_alloc[i].format==format2A)||(DCI_pdu->dci_alloc[i].format==format1)|| (DCI_pdu->dci_alloc[i].format==format1A) ){

      input_buffer_length = phy_vars_eNB->dlsch_eNB[k][0]->harq_processes[harq_pid]->TBS/8;
      input_buffer[k] = (unsigned char *)malloc(input_buffer_length+4);
	  input_buffer_0[k] = (unsigned char *)malloc(input_buffer_length+4);
      memset(input_buffer[k],0,input_buffer_length+4);
	  memset(input_buffer_0[k],0,input_buffer_length+4);

	
	
	//For MIMO TM3/4
   //LOG_I(PHY,input_buffer_length TB0 = %d ,input_buffer_length TB1 = %d \n",phy_vars_eNB->dlsch_eNB[k][0]->harq_processes[harq_pid]->TBS/8,phy_vars_eNB->dlsch_eNB[k][1]->harq_processes[harq_pid]->TBS/8 );	
/*
		for (ii=0;ii<input_buffer_length;ii++) {
			//printf("ii = %d\n",ii);
			input_buffer[k][ii]= (unsigned char)(taus()&0xff);
			input_buffer_0[k][ii]= (unsigned char)(taus()&0xff);
			if(ii==0){
				input_buffer[k][ii]= 0x4f;
			    input_buffer_0[k][ii]= 0x4c;
			}
			if(ii==1){
				input_buffer[k][ii]= 0x00;
                input_buffer_0[k][ii]= 0x4e;
			}
			if(ii==2){
				input_buffer[k][ii]= 0xc0;
				input_buffer_0[k][ii]= 0x6e;
			}
			if(ii==3){
				input_buffer[k][ii]= 0xce;
				input_buffer_0[k][ii]= 0x4f;
			}
			if(ii==4){
				input_buffer[k][ii]= 0x6c;
				input_buffer_0[k][ii]= 0x2c;
			}
			if(ii==5){
				input_buffer[k][ii]= 0x00;
				input_buffer_0[k][ii]= 0x03;
			}
			if(ii==6){
				input_buffer[k][ii]= 0x0c;
				input_buffer_0[k][ii]= 0x05;
			}
		}
		
*/


		for (ii=0;ii<input_buffer_length;ii++) {
			pDlschDataPdu->Data[dlinx][ii]= input_buffer[k][ii];          // FAPI Data
			//pDlschDataPdu->Data[dlinx+1][ii]= input_buffer_0[k][ii];          // FAPI Data
		   //LOG_I(PHY,Data2 = %x\n",pDlschDataPdu->Data[1][ii]);
		}
		
}



  #endif              // TEST MIMO
	 
	 
// ============= DLSCH_pdu data==================	  
// By CFL 2015/04/21
	DLSCH_pdu =  pDlschDataPdu->Data[dlinx]; // FAPI Data       2015/07/16
	DLSCH_pdu_1 = pDlschDataPdu->Data[dlinx+1];
// ===========================================


   if ((DCI_pdu->dci_alloc[i].format==format2)  || (DCI_pdu->dci_alloc[i].format==format2A)) {
   
   // PRINT TM3/TM4
   /*
   	        //pDlschPduConfig = &(pTxVector->dlschConfig.dlschPdu[dlinx]);             // CFL2015/09/22
			//pDlschPduConfig->DlschLength,
            LOG_I(PHY,"PHY TBS1 = %d, Nl = %d, mcs1 = %d\n",
				phy_vars_eNB->dlsch_eNB[i][0]->harq_processes[harq_pid]->TBS/8,
				phy_vars_eNB->dlsch_eNB[i][0]->harq_processes[pDciPdu->harqProc]->Nl,
				phy_vars_eNB->dlsch_eNB[i][0]->harq_processes[pDciPdu->harqProc]->mcs);
				
				
   	       //pDlschPduConfig = &(pTxVector->dlschConfig.dlschPdu[dlinx+1]);            // CFL2015/09/22
			
			LOG_I(PHY,"PHY TBS2 = %d, Nl = %d, mcs2 = %d\n",
				phy_vars_eNB->dlsch_eNB[i][1]->harq_processes[harq_pid]->TBS/8,
				phy_vars_eNB->dlsch_eNB[i][1]->harq_processes[pDciPdu->harqProc]->Nl,
				phy_vars_eNB->dlsch_eNB[i][1]->harq_processes[pDciPdu->harqProc]->mcs);
*/
 
    		/*
			for (ii=0;ii<phy_vars_eNB->dlsch_eNB[i][0]->harq_processes[harq_pid]->TBS/8;ii++) {
				LOG_I(PHY,"Data1 = %x\n",pDlschDataPdu->Data[dlinx][ii]);
		    }
    		for (ii=0;ii<phy_vars_eNB->dlsch_eNB[i][1]->harq_processes[harq_pid]->TBS/8;ii++) {
				LOG_I(PHY,"Data2 = %x\n",pDlschDataPdu->Data[dlinx+1][ii]);
		    }
			*/
			
}
// ===========================================

//===========================================
//////////////////////////// PDSCH encoding ////////////////////////////////  
//===========================================

        phy_vars_eNB->dlsch_eNB[i][0]->harq_processes[pDciPdu->harqProc]->round = 0;             //  Don't know why will not 0;   CFL  2015/05/22
	
	clock_gettime(CLOCK_REALTIME, &encoding_start);
    FILE *encoding = fopen("encoding.txt", "a");
 	if (dlsch_encoding(DLSCH_pdu,                                            // DLSCH data
			&phy_vars_eNB->lte_frame_parms,
			num_pdcch_symbols,
			phy_vars_eNB->dlsch_eNB[i][0],
			pTxVector->txFrameNum,                                             // frame number
			subframe,                                                                    // subframe number
			&phy_vars_eNB->dlsch_rate_matching_stats,
			&phy_vars_eNB->dlsch_turbo_encoding_stats,
			&phy_vars_eNB->dlsch_interleaving_stats)<0) 
	exit(-1);

       clock_gettime(CLOCK_REALTIME, &encoding_end);
       long double accum_encoding = ((long double)encoding_end.tv_sec - encoding_start.tv_sec ) * 1000.0 + ((long double)encoding_end.tv_nsec - encoding_start.tv_nsec ) / MS;
       if (subframe == 1) {
           LOG_I(PHY,"Encoding: %Lf\n", accum_encoding);
		   fprintf(encoding, "%Lf\n", accum_encoding);
       }
	   fclose(encoding);
//============================================
//////////////////////////// PDSCH scrambling ////////////////////////////////  
//============================================
 	dlsch_scrambling(&phy_vars_eNB->lte_frame_parms,
								0,                                                //mbsfn_flag
								phy_vars_eNB->dlsch_eNB[i][0],
								coded_bits_per_codeword,            // number of bits
								0,                                                // codeword q (0, 1)
								subframe<<1);	                           // no. slot


//============== TB2 (MIMO TM3 & TM4)================= 
if((DCI_pdu->dci_alloc[i].format==format2)  || (DCI_pdu->dci_alloc[i].format==format2A)) {
	
	    phy_vars_eNB->dlsch_eNB[i][1]->harq_processes[pDciPdu->harqProc]->round = 0;             //  Don't know why will not 0;   CFL  2015/09/22
		coded_bits_per_codeword = get_G(&phy_vars_eNB->lte_frame_parms,                                                       // TB1
						phy_vars_eNB->dlsch_eNB[i][1]->harq_processes[pDciPdu->harqProc]->nb_rb,
					        phy_vars_eNB->dlsch_eNB[i][1]->harq_processes[pDciPdu->harqProc]->rb_alloc,
						get_Qm(phy_vars_eNB->dlsch_eNB[i][1]->harq_processes[pDciPdu->harqProc]->mcs),           // TB0 and TB1 maybe will different
					        phy_vars_eNB->dlsch_eNB[i][1]->harq_processes[pDciPdu->harqProc]->Nl,
						num_pdcch_symbols,
					        phy_vars_eNB->proc[sched_subframe].frame_tx,
						subframe);

 	if (dlsch_encoding(DLSCH_pdu_1,                                            // DLSCH data
			&phy_vars_eNB->lte_frame_parms,
			num_pdcch_symbols,
			phy_vars_eNB->dlsch_eNB[i][1],
			pTxVector->txFrameNum,                                             // frame number
			subframe,                                                                    // subframe number
			&phy_vars_eNB->dlsch_rate_matching_stats,
			&phy_vars_eNB->dlsch_turbo_encoding_stats,
			&phy_vars_eNB->dlsch_interleaving_stats
			)<0) 
	exit(-1);
	
 	dlsch_scrambling(&phy_vars_eNB->lte_frame_parms,
			 0,                                                //mbsfn_flag
			 phy_vars_eNB->dlsch_eNB[i][1],
		         coded_bits_per_codeword,            // number of bits
			 1,                                                // codeword q (0, 1)
			 subframe << 1);	                           // no. slot
}


//============================================
//////////////////////////// PDSCH modulation ////////////////////////////////  
//============================================
clock_gettime(CLOCK_REALTIME, &modulation_start);
FILE *modulation = fopen("modulation.txt", "a");
re_allocated = dlsch_modulation(phy_vars_eNB->lte_eNB_common_vars.txdataF[0],
					      AMP,
					      subframe,                          				  // no. subframe
					      &phy_vars_eNB->lte_frame_parms,
					      num_pdcch_symbols,                  	  // CFI
					      phy_vars_eNB->dlsch_eNB[i][0],        // dlsch0
						  phy_vars_eNB->dlsch_eNB[i][1]);       // dlsch1
   clock_gettime(CLOCK_REALTIME, &modulation_end);
       long double accum_modulation = ((long double)modulation_end.tv_sec - modulation_start.tv_sec ) * 1000.0 + ((long double)modulation_end.tv_nsec - modulation_start.tv_nsec ) / MS;
       if (subframe == 1) {
           LOG_I(PHY,"Modulation: %Lf\n", accum_modulation);
		   fprintf(modulation, "%Lf\n", accum_modulation);
       	}
	    fclose(modulation);
   if ((DCI_pdu->dci_alloc[i].format==format2)  || (DCI_pdu->dci_alloc[i].format==format2A)) 
    dlinx+=2;
   else
    dlinx++;
   
      }                         // for RA procedure if

}                              // loop for DCI number

   #endif              // Schedule of III PDSCH

 
 // =================== HI_Flag =========================
 //usleep(100); //joe 20160415 omit the HI_DCI0 latency 
 
 // ============= Generate PDCCH and PCFICH===================
     if (pTxVector->DciNum==0){                                     // if has DCI
		DCI_pdu->Num_ue_spec_dci = 0;
		DCI_pdu->Num_common_dci = 0;
	 }
	 
// ================== DCI_UL DCI ========================
int dciULidx = 0,hiidx = 0;                                           // MUS 2015/09/17


 for (dciULidx = 0; dciULidx <pHiDciVectorReq->dciUlConfig.numOfPdu; dciULidx++){
	 
 pDciUlPduConfig = &(pHiDciVectorReq->dciUlConfig.dciUlPduConfig[dciULidx]);             //index user number
 aggregationLevel = log(pDciUlPduConfig->aggregationLevel)/log(2);  // must to be log 
  
 #if 0
 LOG_I(PHY,"=================Subframe = %d=====================\n",subframe);	
 LOG_I(PHY,"Number of DCI format 0 = %d,User %d DCI format 0 \n", pHiDciVectorReq->dciUlConfig.numOfPdu,dciULidx);
 LOG_I(PHY,"format = %d,cceStart = %d,aggregationLevel= %d,rnti= %d,rbStart = %d,rbNum = %d\nmcs = %d,dmrs = %d,freqHoppingFlag = %d,freqHoppingBits = %d\nndi = %d,tpc = %d,cqiRequest = %d,ulIndex = %d,,tpcBitmap = %d\n",
		pDciUlPduConfig->format,
		pDciUlPduConfig->cceStart ,
		pDciUlPduConfig->aggregationLevel,
		pDciUlPduConfig->rnti,
		pDciUlPduConfig->rbStart ,
		pDciUlPduConfig->rbNum ,
		pDciUlPduConfig->mcs,
		pDciUlPduConfig->dmrs,
		pDciUlPduConfig->freqHoppingFlag,
		pDciUlPduConfig->freqHoppingBits,
		pDciUlPduConfig->ndi,
		pDciUlPduConfig->tpc,
		pDciUlPduConfig->cqiRequest,
		pDciUlPduConfig->ulIndex,
		pDciUlPduConfig->tpcBitmap);
		
LOG_I(PHY,"==================================================\n\n");	
	 
#endif	 


 // ============ DCI format 0  ================
 if (pHiDciVectorReq->dciUlConfig.numOfPdu!=0){
		memset(&(ULSCH_alloc_pdu),0, sizeof(ULSCH_alloc_pdu)); //joe 20160427
	//LOG_I(PHY,DCI Number = %d\n\n",i);
	if(pDciUlPduConfig->format == format0){
	//LOG_I(PHY,"(%d,%d) use DCI Format : 0\n",pTxVector->txFrameNum,subframe);
        switch (phy_vars_eNB->lte_frame_parms.N_RB_DL) {
        case 50:
            DCI_pdu->dci_alloc[i+dciULidx].dci_length = sizeof_DCI0_10MHz_FDD_t;
            DCI_pdu->dci_alloc[i+dciULidx].nCCE       = pDciUlPduConfig->cceStart ;
            DCI_pdu->dci_alloc[i+dciULidx].ra_flag      = 0;
            DCI_pdu->dci_alloc[i+dciULidx].L              = aggregationLevel;
            DCI_pdu->dci_alloc[i+dciULidx].rnti           = pDciUlPduConfig->rnti;;
            
            DLSCH_RB_ALLOC = computeRIV(50,pDciUlPduConfig->rbStart ,pDciUlPduConfig->rbNum);

            dci_length_bytes = sizeof(DCI0_10MHz_FDD_t);

            ((DCI0_10MHz_FDD_t *)&ULSCH_alloc_pdu)->type                 = 0;
            ((DCI0_10MHz_FDD_t *)&ULSCH_alloc_pdu)->hopping              = pDciUlPduConfig->freqHoppingBits;//0
            ((DCI0_10MHz_FDD_t *)&ULSCH_alloc_pdu)->rballoc              = DLSCH_RB_ALLOC;
            ((DCI0_10MHz_FDD_t *)&ULSCH_alloc_pdu)->mcs                  = pDciUlPduConfig->mcs;
            ((DCI0_10MHz_FDD_t *)&ULSCH_alloc_pdu)->ndi                   = pDciUlPduConfig->ndi;
            ((DCI0_10MHz_FDD_t *)&ULSCH_alloc_pdu)->TPC                 = pDciUlPduConfig->tpc;
            ((DCI0_10MHz_FDD_t *)&ULSCH_alloc_pdu)->cshift                = pDciUlPduConfig->dmrs;
            ((DCI0_10MHz_FDD_t *)&ULSCH_alloc_pdu)->cqi_req             = pDciUlPduConfig->cqiRequest;
            
            memcpy((void*)&DCI_pdu->dci_alloc[i+dciULidx].dci_pdu[0],(void *)&ULSCH_alloc_pdu,sizeof(DCI0_10MHz_FDD_t));
        break;
        case 100:
            DCI_pdu->dci_alloc[i+dciULidx].dci_length = sizeof_DCI0_20MHz_FDD_t;
            DCI_pdu->dci_alloc[i+dciULidx].nCCE       = pDciUlPduConfig->cceStart ;
            DCI_pdu->dci_alloc[i+dciULidx].ra_flag      = 0;
            DCI_pdu->dci_alloc[i+dciULidx].L              = aggregationLevel;
            DCI_pdu->dci_alloc[i+dciULidx].rnti           = pDciUlPduConfig->rnti;;
            
            DLSCH_RB_ALLOC = computeRIV(100,pDciUlPduConfig->rbStart ,pDciUlPduConfig->rbNum);  // computeRIV(nRB,RBstart,Lcrbs)

            dci_length_bytes = sizeof(DCI0_20MHz_FDD_t);

            ((DCI0_20MHz_FDD_t *)&ULSCH_alloc_pdu)->type                 = 0;
            ((DCI0_20MHz_FDD_t *)&ULSCH_alloc_pdu)->hopping              = pDciUlPduConfig->freqHoppingBits;//0
            ((DCI0_20MHz_FDD_t *)&ULSCH_alloc_pdu)->rballoc              = DLSCH_RB_ALLOC;
            ((DCI0_20MHz_FDD_t *)&ULSCH_alloc_pdu)->mcs                  = pDciUlPduConfig->mcs;
            ((DCI0_20MHz_FDD_t *)&ULSCH_alloc_pdu)->ndi                  = pDciUlPduConfig->ndi;
            ((DCI0_20MHz_FDD_t *)&ULSCH_alloc_pdu)->TPC                  = pDciUlPduConfig->tpc;
            ((DCI0_20MHz_FDD_t *)&ULSCH_alloc_pdu)->cshift               = pDciUlPduConfig->dmrs;
            ((DCI0_20MHz_FDD_t *)&ULSCH_alloc_pdu)->cqi_req              = pDciUlPduConfig->cqiRequest;
           
            memcpy((void*)&DCI_pdu->dci_alloc[i+dciULidx].dci_pdu[0],(void *)&ULSCH_alloc_pdu,sizeof(DCI0_20MHz_FDD_t));

		break;
		
        }	
	DCI_pdu->Num_ue_spec_dci ++;              //20150918
	
	 }
 
   }
}

// ================= Generate PCFICH and PDCCH ====================

//LOG_I(PHY,"Num_ue_spec_dci = %d,  Num_common_dci = %d\n",DCI_pdu->Num_ue_spec_dci,DCI_pdu->Num_common_dci);
  if (abstraction_flag == 0) {
	  
	if (num_pdcch_symbols!=2){
		LOG_I(PHY,"num_pdcch_symbols = %d in subframe = %d\n", num_pdcch_symbols,subframe);  //Check if CFI =3
    }  
      num_pdcch_symbols = generate_dci_top(DCI_pdu->Num_ue_spec_dci,
					   DCI_pdu->Num_common_dci,
					   DCI_pdu->dci_alloc,
					   0,
					   AMP,
					   &phy_vars_eNB->lte_frame_parms,
					   phy_vars_eNB->lte_eNB_common_vars.txdataF[0],
					   subframe);
					   
  }
  
// ==================== Generate PHICH ====================
  for (hiidx = 0; hiidx <pHiDciVectorReq->hiConfig.numOfPdu; hiidx++){
	pHiPduConfig = &(pHiDciVectorReq->hiConfig.hiPduConfig[hiidx]);              // hiidx   

 #if 0
 {
 LOG_I(PHY,"=================Subframe = %d=====================\n",subframe);	
 LOG_I(PHY,"Number of PHICH = %d,User %d ACK/NACK \n", pHiDciVectorReq->hiConfig.numOfPdu,hiidx);
 LOG_I(PHY,"RBstart = %d,dmrs = %d,hiValue= %d,iPhich = %d,txPower = %d\n",
		  pHiPduConfig->ulLowestRb,
		  pHiPduConfig->dmrs,
	  	  pHiPduConfig->hiValue,
		  pHiPduConfig->iPhich,
		  pHiPduConfig->txPower );
		
 LOG_I(PHY,"==================================================\n\n");	
	 
 }
#endif
harq_pid = 0;  //joe 20160415 phich harqpid

phy_vars_eNB->ulsch_eNB[hiidx]->harq_processes[harq_pid]->phich_active = 1;
//phy_vars_eNB->ulsch_eNB[hiidx]->rnti =  DCI_pdu->dci_alloc[0].rnti;                      // i has bug need to fix
phy_vars_eNB->ulsch_eNB[hiidx]->harq_processes[harq_pid]->first_rb = pHiPduConfig->ulLowestRb;
phy_vars_eNB->ulsch_eNB[hiidx]->harq_processes[harq_pid]->n_DMRS = pHiPduConfig->dmrs;
phy_vars_eNB->ulsch_eNB[hiidx]->harq_processes[harq_pid]->phich_ACK = pHiPduConfig->hiValue;

// ==================== Generate PHICH ====================
	if (pHiDciVectorReq->hiConfig.numOfPdu!=0) {
    generate_phich_top(phy_vars_eNB,
		       sched_subframe,
		       AMP,
		       0,
		       pHiDciVectorReq->hiConfig.numOfPdu);
   }
   
 }

 // ===================== Reset DCI =======================
//for(i=0 ; i<pTxVector->DciNum ; i++){	           // loop for DCI number  
for(i=0 ; i<(pTxVector->DciNum + dciULidx) ; i++){	           // loop for DCI number  20150918
   memset(&(DCI_pdu->dci_alloc[i]),0, sizeof(DCI_ALLOC_t)); 
}
memset(pHiDciVectorReq,0,sizeof(tMacaHiDciVectorReq));
 // =================== IFFT and CP insert =================== 
# if IQ_TEST
  if (abstraction_flag==0) {
    start_meas(&phy_vars_eNB->ofdm_mod_stats);
    do_OFDM_mod(phy_vars_eNB->lte_eNB_common_vars.txdataF[0],
		phy_vars_eNB->lte_eNB_common_vars.txdata[0],
		phy_vars_eNB->proc[sched_subframe].frame_tx,subframe<<1,
		&phy_vars_eNB->lte_frame_parms);
    do_OFDM_mod(phy_vars_eNB->lte_eNB_common_vars.txdataF[0],
		phy_vars_eNB->lte_eNB_common_vars.txdata[0],
		phy_vars_eNB->proc[sched_subframe].frame_tx,1+(subframe<<1),
		&phy_vars_eNB->lte_frame_parms);
    stop_meas(&phy_vars_eNB->ofdm_mod_stats);
  }

#endif

#if MEASTIME	
  
clock_gettime(CLOCK_REALTIME, &requestEnd);      // Time end
double accum = ( requestEnd.tv_sec - requestStart.tv_sec ) + ( requestEnd.tv_nsec - requestStart.tv_nsec ) / MS;
printf( "Generated a subframe %d time = %lf ms\n\n",subframe, accum );	

#endif
  
  stop_meas(&phy_vars_eNB->phy_proc_tx);
#ifdef MEASURETXTIME
  time_end = rt_get_time_ns();
  if((time_end-time_start>=780000) && (time_end-time_start<=2000000) )
    LOG_I(PHY,"(%d,%d)PHY TX process time exceed 780us, time(%llu)\n",pTxVector->txFrameNum,subframe,time_end-time_start);
#endif
}


void process_Msg3(PHY_VARS_eNB *phy_vars_eNB,uint8_t sched_subframe,uint8_t UE_id, uint8_t harq_pid) {
  // this prepares the demodulation of the first PUSCH of a new user, containing Msg3

  int subframe = phy_vars_eNB->proc[sched_subframe].subframe_rx;
  int frame = phy_vars_eNB->proc[sched_subframe].frame_rx;

  LOG_D(PHY,"[eNB %d][RAPROC] frame %d : subframe %d : process_Msg3 UE_id %d (active %d, subframe %d, frame %d)\n",
	phy_vars_eNB->Mod_id,
	frame,subframe,
	UE_id,phy_vars_eNB->ulsch_eNB[(uint32_t)UE_id]->Msg3_active,
	phy_vars_eNB->ulsch_eNB[(uint32_t)UE_id]->Msg3_subframe,
	phy_vars_eNB->ulsch_eNB[(uint32_t)UE_id]->Msg3_frame);
  phy_vars_eNB->ulsch_eNB[(uint32_t)UE_id]->Msg3_flag = 0;
  
  if ((phy_vars_eNB->ulsch_eNB[(uint32_t)UE_id]->Msg3_active == 1) && 
      (phy_vars_eNB->ulsch_eNB[(uint32_t)UE_id]->Msg3_subframe == subframe) &&
      (phy_vars_eNB->ulsch_eNB[(uint32_t)UE_id]->Msg3_frame == (uint32_t)frame))   {
    
    //    harq_pid = 0;
    
    phy_vars_eNB->ulsch_eNB[(uint32_t)UE_id]->Msg3_active = 0;
    phy_vars_eNB->ulsch_eNB[(uint32_t)UE_id]->Msg3_flag = 1;
    phy_vars_eNB->ulsch_eNB[(uint32_t)UE_id]->harq_processes[harq_pid]->subframe_scheduling_flag=1;
    LOG_D(PHY,"[eNB %d][RAPROC] frame %d, subframe %d: Setting subframe_scheduling_flag (Msg3) for UE %d\n",
	  phy_vars_eNB->Mod_id,
	  frame,subframe,UE_id);
  }
}


// This function retrieves the harq_pid of the corresponding DLSCH process
// and updates the error statistics of the DLSCH based on the received ACK
// info from UE along with the round index.  It also performs the fine-grain
// rate-adaptation based on the error statistics derived from the ACK/NAK process

void process_HARQ_feedback(uint8_t UE_id, 
			   uint8_t sched_subframe, 
			   PHY_VARS_eNB *phy_vars_eNB,
			   uint8_t pusch_flag, 
			   uint8_t *pucch_payload, 
			   uint8_t pucch_sel,
			   uint8_t SR_payload) {

  uint8_t dl_harq_pid[8],dlsch_ACK[8],dl_subframe;
  LTE_eNB_DLSCH_t *dlsch             =  phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0];
  LTE_eNB_UE_stats *ue_stats         =  &phy_vars_eNB->eNB_UE_stats[(uint32_t)UE_id];
  LTE_DL_eNB_HARQ_t *dlsch_harq_proc;
  uint8_t subframe_m4,M,m;
  int mp;
  int all_ACKed=1,nb_alloc=0,nb_ACK=0;
  int frame = phy_vars_eNB->proc[sched_subframe].frame_rx;
  int subframe = phy_vars_eNB->proc[sched_subframe].subframe_rx;
  int harq_pid = subframe2harq_pid( &phy_vars_eNB->lte_frame_parms,frame,subframe);

  if (phy_vars_eNB->lte_frame_parms.frame_type == FDD){ //FDD
    subframe_m4 = (subframe<4) ? subframe+6 : subframe-4;

    dl_harq_pid[0] = dlsch->harq_ids[subframe_m4];
    M=1;
    if (pusch_flag == 1)
      dlsch_ACK[0] = phy_vars_eNB->ulsch_eNB[(uint8_t)UE_id]->harq_processes[harq_pid]->o_ACK[0];
    else
      dlsch_ACK[0] = pucch_payload[0];
    LOG_D(PHY,"[eNB %d] Frame %d: Received ACK/NAK %d for subframe %d\n",phy_vars_eNB->Mod_id,
	  frame,dlsch_ACK[0],subframe_m4);
  }
  else {  // TDD Handle M=1,2 cases only
    
    M=ul_ACK_subframe2_M(&phy_vars_eNB->lte_frame_parms,
			 subframe);
    // Now derive ACK information for TDD
    if (pusch_flag == 1) { // Do PUSCH ACK/NAK first
      // detect missing DAI
      //FK: this code is just a guess
      //RK: not exactly, yes if scheduled from PHICH (i.e. no DCI format 0)
      //    otherwise, it depends on how many of the PDSCH in the set are scheduled, we can leave it like this,
      //    but we have to adapt the code below.  For example, if only one out of 2 are scheduled, only 1 bit o_ACK is used

      dlsch_ACK[0] = phy_vars_eNB->ulsch_eNB[(uint8_t)UE_id]->harq_processes[harq_pid]->o_ACK[0];
      dlsch_ACK[1] = (phy_vars_eNB->pucch_config_dedicated[UE_id].tdd_AckNackFeedbackMode == bundling)?phy_vars_eNB->ulsch_eNB[(uint8_t)UE_id]->harq_processes[harq_pid]->o_ACK[0]:phy_vars_eNB->ulsch_eNB[(uint8_t)UE_id]->harq_processes[harq_pid]->o_ACK[1];
      //      printf("UE %d: ACK %d,%d\n",UE_id,dlsch_ACK[0],dlsch_ACK[1]);
    }

    else {  // PUCCH ACK/NAK
      if ((SR_payload == 1)&&(pucch_sel!=2)) {  // decode Table 7.3 if multiplexing and SR=1 
	nb_ACK = 0;
	if (M == 2) {
	  if ((pucch_payload[0] == 1) && (pucch_payload[1] == 1)) // b[0],b[1]
	    nb_ACK = 1;
	  else if ((pucch_payload[0] == 1) && (pucch_payload[1] == 0))
	    nb_ACK = 2;
	}
	else if (M == 3) {
	  if ((pucch_payload[0] == 1) && (pucch_payload[1] == 1)) 
	    nb_ACK = 1;
	  else if ((pucch_payload[0] == 1) && (pucch_payload[1] == 0))
	    nb_ACK = 2;
	  else if ((pucch_payload[0] == 0) && (pucch_payload[1] == 1))
	    nb_ACK = 3;
	}
      }
      else if (pucch_sel == 2) {  // bundling or M=1
	//	printf("*** (%d,%d)\n",pucch_payload[0],pucch_payload[1]);
	dlsch_ACK[0] = pucch_payload[0];
	dlsch_ACK[1] = pucch_payload[0];
      }
      else {  // multiplexing with no SR, this is table 10.1
	if (M==1) 
	  dlsch_ACK[0] = pucch_payload[0];
	else if (M==2) {
	  if (((pucch_sel == 1) && (pucch_payload[0] == 1) && (pucch_payload[1] == 1)) ||
	      ((pucch_sel == 0) && (pucch_payload[0] == 0) && (pucch_payload[1] == 1)))
	    dlsch_ACK[0] = 1;
	  else
	    dlsch_ACK[0] = 0;

	  if (((pucch_sel == 1) && (pucch_payload[0] == 1) && (pucch_payload[1] == 1)) ||
	      ((pucch_sel == 1) && (pucch_payload[0] == 0) && (pucch_payload[1] == 0)))
	    dlsch_ACK[1] = 1;
	  else
	    dlsch_ACK[1] = 0;
	}
      }
    }
  }

  // handle case where positive SR was transmitted with multiplexing
  if ((SR_payload == 1)&&(pucch_sel!=2)&&(pusch_flag == 0)) {
    nb_alloc = 0;
    for (m=0;m<M;m++) {
      dl_subframe = ul_ACK_subframe2_dl_subframe(&phy_vars_eNB->lte_frame_parms,
						 subframe,
						 m);
      
      if (dlsch->subframe_tx[dl_subframe]==1) 
	nb_alloc++;
    }
    if (nb_alloc == nb_ACK)
      all_ACKed = 1;
    else 
      all_ACKed = 0;

    //    printf("nb_alloc %d, all_ACKed %d\n",nb_alloc,all_ACKed);
  }


  for (m=0,mp=-1;m<M;m++) {

    dl_subframe = ul_ACK_subframe2_dl_subframe(&phy_vars_eNB->lte_frame_parms,
					       subframe,
					       m);

    if (dlsch->subframe_tx[dl_subframe]==1) {
      if (pusch_flag == 1)
	mp++;
      else
	mp = m;

      dl_harq_pid[m]     = dlsch->harq_ids[dl_subframe];

      if ((pucch_sel != 2)&&(pusch_flag == 0)) { // multiplexing
	if ((SR_payload == 1)&&(all_ACKed == 1))
	  dlsch_ACK[m] = 1;
	else	
	  dlsch_ACK[m] = 0;
      }
      if (dl_harq_pid[m]<dlsch->Mdlharq) {
	dlsch_harq_proc = dlsch->harq_processes[dl_harq_pid[m]];
#ifdef DEBUG_PHY_PROC	
	LOG_D(PHY,"[eNB %d][PDSCH %x/%d] subframe %d, status %d, round %d (mcs %d, rv %d, TBS %d)\n",phy_vars_eNB->Mod_id,
	      dlsch->rnti,dl_harq_pid[m],dl_subframe,
	      dlsch_harq_proc->status,dlsch_harq_proc->round,
	      dlsch->harq_processes[dl_harq_pid[m]]->mcs,
	      dlsch->harq_processes[dl_harq_pid[m]]->rvidx,
	      dlsch->harq_processes[dl_harq_pid[m]]->TBS);
	if (dlsch_harq_proc->status==DISABLED)
	  LOG_E(PHY,"dlsch_harq_proc is disabled? \n");
#endif
	if ((dl_harq_pid[m]<dlsch->Mdlharq) &&
	    (dlsch_harq_proc->status == ACTIVE)) {
	  // dl_harq_pid of DLSCH is still active
	  
	  //	  msg("[PHY] eNB %d Process %d is active (%d)\n",phy_vars_eNB->Mod_id,dl_harq_pid[m],dlsch_ACK[m]);
	  if ( dlsch_ACK[mp]==0) {
	    // Received NAK 
#ifdef DEBUG_PHY_PROC	
	    LOG_D(PHY,"[eNB %d][PDSCH %x/%d] M = %d, m= %d, mp=%d NAK Received in round %d, requesting retransmission\n",phy_vars_eNB->Mod_id,
		  dlsch->rnti,dl_harq_pid[m],M,m,mp,dlsch_harq_proc->round);
#endif
	    
	    if (dlsch_harq_proc->round == 0) 
	      ue_stats->dlsch_NAK_round0++;
	    ue_stats->dlsch_NAK[dl_harq_pid[m]][dlsch_harq_proc->round]++;

	    
	    // then Increment DLSCH round index 
	    dlsch_harq_proc->round++;
	    
	    if (dlsch_harq_proc->round == dlsch->Mdlharq) {
	      // This was the last round for DLSCH so reset round and increment l2_error counter
#ifdef DEBUG_PHY_PROC	
	      LOG_W(PHY,"[eNB %d][PDSCH %x/%d] DLSCH retransmissions exhausted, dropping packet\n",phy_vars_eNB->Mod_id,
		    dlsch->rnti,dl_harq_pid[m]);
#endif
	      dlsch_harq_proc->round = 0;
	      ue_stats->dlsch_l2_errors[dl_harq_pid[m]]++;
	      dlsch_harq_proc->status = SCH_IDLE;
	      dlsch->harq_ids[dl_subframe] = dlsch->Mdlharq;
	    }
	  }
	  else {
#ifdef DEBUG_PHY_PROC	
	    LOG_D(PHY,"[eNB %d][PDSCH %x/%d] ACK Received in round %d, resetting process\n",phy_vars_eNB->Mod_id,
		  dlsch->rnti,dl_harq_pid[m],dlsch_harq_proc->round);
#endif
	    ue_stats->dlsch_ACK[dl_harq_pid[m]][dlsch_harq_proc->round]++;

	    // Received ACK so set round to 0 and set dlsch_harq_pid IDLE
	    dlsch_harq_proc->round  = 0;
	    dlsch_harq_proc->status = SCH_IDLE; 
	    dlsch->harq_ids[dl_subframe] = dlsch->Mdlharq;

	    ue_stats->total_TBS = ue_stats->total_TBS + 
	      phy_vars_eNB->dlsch_eNB[(uint8_t)UE_id][0]->harq_processes[dl_harq_pid[m]]->TBS;
	    /*
	      ue_stats->total_transmitted_bits = ue_stats->total_transmitted_bits +
	      phy_vars_eNB->dlsch_eNB[(uint8_t)UE_id][0]->harq_processes[dl_harq_pid[m]]->TBS;
	    */
	  }
	  
	  // Do fine-grain rate-adaptation for DLSCH 
	  if (ue_stats->dlsch_NAK_round0 > dlsch->error_threshold) {
	    if (ue_stats->dlsch_mcs_offset == 1)
	      ue_stats->dlsch_mcs_offset=0;
	    else
	      ue_stats->dlsch_mcs_offset=-1;
	  }
#ifdef DEBUG_PHY_PROC	  
	  LOG_D(PHY,"[process_HARQ_feedback] Frame %d Setting round to %d for pid %d (subframe %d)\n",frame,
		dlsch_harq_proc->round,dl_harq_pid[m],subframe);
#endif
	  
	  // Clear NAK stats and adjust mcs offset
	  // after measurement window timer expires
	  if ((ue_stats->dlsch_sliding_cnt == dlsch->ra_window_size) ) {
	    if ((ue_stats->dlsch_mcs_offset == 0) && (ue_stats->dlsch_NAK_round0 < 2))
	      ue_stats->dlsch_mcs_offset = 1;
	    if ((ue_stats->dlsch_mcs_offset == 1) && (ue_stats->dlsch_NAK_round0 > 2))
	      ue_stats->dlsch_mcs_offset = 0;
	    if ((ue_stats->dlsch_mcs_offset == 0) && (ue_stats->dlsch_NAK_round0 > 2))
	      ue_stats->dlsch_mcs_offset = -1;
	    if ((ue_stats->dlsch_mcs_offset == -1) && (ue_stats->dlsch_NAK_round0 < 2))
	      ue_stats->dlsch_mcs_offset = 0;
	    
	    ue_stats->dlsch_NAK_round0 = 0;
	    ue_stats->dlsch_sliding_cnt = 0;
	  }
	  
	  
	}
      }
    }
  }
}

void get_n1_pucch_eNB(PHY_VARS_eNB *phy_vars_eNB,
		      uint8_t UE_id,
		      uint8_t sched_subframe,
		      int16_t *n1_pucch0,
		      int16_t *n1_pucch1,
		      int16_t *n1_pucch2,
		      int16_t *n1_pucch3) {

  LTE_DL_FRAME_PARMS *frame_parms=&phy_vars_eNB->lte_frame_parms;
  uint8_t nCCE0,nCCE1;
  int sf;
  int frame = phy_vars_eNB->proc[sched_subframe].frame_rx;
  int subframe = phy_vars_eNB->proc[sched_subframe].subframe_rx;

  if (frame_parms->frame_type == FDD ) {
    sf = (subframe<4) ? (subframe+6) : (subframe-4); 
    //    printf("n1_pucch_eNB: subframe %d, nCCE %d\n",sf,phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[sf]);

    if (phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[sf]>0) {
      *n1_pucch0 = frame_parms->pucch_config_common.n1PUCCH_AN + phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[sf];
      *n1_pucch1 = -1;
    }
    else {
      *n1_pucch0 = -1;
      *n1_pucch1 = -1;
    }
  }
  else {

    switch (frame_parms->tdd_config) {
    case 1:  // DL:S:UL:UL:DL:DL:S:UL:UL:DL
      if (subframe == 2) {  // ACK subframes 5 and 6
	/*	if (phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[6]>0) {
		nCCE1 = phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[6];
		*n1_pucch1 = get_Np(frame_parms->N_RB_DL,nCCE1,1) + nCCE1 + frame_parms->pucch_config_common.n1PUCCH_AN; 
		}
		else
		*n1_pucch1 = -1;*/

	if (phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[5]>0) {
	  nCCE0 = phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[5];
	  *n1_pucch0 = get_Np(frame_parms->N_RB_DL,nCCE0,0) + nCCE0+ frame_parms->pucch_config_common.n1PUCCH_AN; 
	}
	else
	  *n1_pucch0 = -1;

	*n1_pucch1 = -1;	
      }
      else if (subframe == 3) {   // ACK subframe 9
	
	if (phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[9]>0) {
	  nCCE0 = phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[9];
	  *n1_pucch0 = get_Np(frame_parms->N_RB_DL,nCCE0,0) + nCCE0 +frame_parms->pucch_config_common.n1PUCCH_AN; 
	}
	else
	  *n1_pucch0 = -1;

	*n1_pucch1 = -1;
	
      }
      else if (subframe == 7) {  // ACK subframes 0 and 1
	//harq_ack[0].nCCE;  
	//harq_ack[1].nCCE;
	if (phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[0]>0) {
	  nCCE0 = phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[0];
	  *n1_pucch0 = get_Np(frame_parms->N_RB_DL,nCCE0,0) + nCCE0 + frame_parms->pucch_config_common.n1PUCCH_AN; 
	}
	else
	  *n1_pucch0 = -1;

	*n1_pucch1 = -1;
      }
      else if (subframe == 8) {   // ACK subframes 4
	//harq_ack[4].nCCE;
	if (phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[4]>0) {
	  nCCE0 = phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[4];
	  *n1_pucch0 = get_Np(frame_parms->N_RB_DL,nCCE0,0) + nCCE0 + frame_parms->pucch_config_common.n1PUCCH_AN; 
	}
	else
	  *n1_pucch0 = -1;

	*n1_pucch1 = -1;
      }
      else {
	LOG_D(PHY,"[eNB %d] frame %d: phy_procedures_lte.c: get_n1pucch, illegal subframe %d for tdd_config %d\n",
	      phy_vars_eNB->Mod_id,
	      frame,
	      subframe,frame_parms->tdd_config);
	return;
      }
      break;
    case 3:  // DL:S:UL:UL:UL:DL:DL:DL:DL:DL
      if (subframe == 2) {  // ACK subframes 5,6 and 1 (S in frame-2), forget about n-11 for the moment (S-subframe)
	if (phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[6]>0) {
	  nCCE1 = phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[6];
	  *n1_pucch1 = get_Np(frame_parms->N_RB_DL,nCCE1,1) + nCCE1 + frame_parms->pucch_config_common.n1PUCCH_AN; 
	}
	else
	  *n1_pucch1 = -1;

	if (phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[5]>0) {
	  nCCE0 = phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[5];
	  *n1_pucch0 = get_Np(frame_parms->N_RB_DL,nCCE0,0) + nCCE0+ frame_parms->pucch_config_common.n1PUCCH_AN; 
	}
	else
	  *n1_pucch0 = -1;
      }
      else if (subframe == 3) {   // ACK subframes 7 and 8
	LOG_D(PHY,"get_n1_pucch_eNB : subframe 3, subframe_tx[7] %d, subframe_tx[8] %d\n",
	      phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[7],phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[8]);

	if (phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[8]>0) {
	  nCCE1 = phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[8];
	  *n1_pucch1 = get_Np(frame_parms->N_RB_DL,nCCE1,1) + nCCE1 + frame_parms->pucch_config_common.n1PUCCH_AN;
	  LOG_D(PHY,"nCCE1 %d, n1_pucch1 %d\n",nCCE1,*n1_pucch1);
	}
	else
	  *n1_pucch1 = -1;

	if (phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[7]>0) {
	  nCCE0 = phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[7];
	  *n1_pucch0 = get_Np(frame_parms->N_RB_DL,nCCE0,0) + nCCE0 +frame_parms->pucch_config_common.n1PUCCH_AN; 
	  LOG_D(PHY,"nCCE0 %d, n1_pucch0 %d\n",nCCE0,*n1_pucch0);
	}
	else
	  *n1_pucch0 = -1;
      }
      else if (subframe == 4) {  // ACK subframes 9 and 0
	if (phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[0]>0) {
	  nCCE1 = phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[0];
	  *n1_pucch1 = get_Np(frame_parms->N_RB_DL,nCCE1,1) + nCCE1 + frame_parms->pucch_config_common.n1PUCCH_AN; 
	}
	else
	  *n1_pucch1 = -1;

	if (phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->subframe_tx[9]>0) {
	  nCCE0 = phy_vars_eNB->dlsch_eNB[(uint32_t)UE_id][0]->nCCE[9];
	  *n1_pucch0 = get_Np(frame_parms->N_RB_DL,nCCE0,0) + nCCE0 +frame_parms->pucch_config_common.n1PUCCH_AN; 
	}
	else
	  *n1_pucch0 = -1;
      }
      else {
	LOG_D(PHY,"[eNB %d] Frame %d: phy_procedures_lte.c: get_n1pucch, illegal subframe %d for tdd_config %d\n",
	      phy_vars_eNB->Mod_id,frame,subframe,frame_parms->tdd_config);
	return;
      }
      break;
    }  // switch tdd_config     
    // Don't handle the case M>2
    *n1_pucch2 = -1;
    *n1_pucch3 = -1;
  }
}


extern int16_t prach_ifft[4][1024*4];
extern tMacaRxDataInd   sRxDataInd[10];
/* joe 20150420 PRACH enable flag*/ 
extern bool RachPresentFlag[10];


void prach_procedures(PHY_VARS_eNB *phy_vars_eNB,uint8_t sched_subframe,uint8_t abstraction_flag) {

  uint16_t preamble_energy_list[64],preamble_delay_list[64];
  uint16_t preamble_max,preamble_energy_max;
  uint16_t i;
  int8_t UE_id;
  int subframe = phy_vars_eNB->proc[sched_subframe].subframe_rx;
  int frame = phy_vars_eNB->proc[sched_subframe].frame_rx;
  uint8_t CC_id = phy_vars_eNB->CC_id;

  memset(&preamble_energy_list[0],0,64*sizeof(uint16_t));
  memset(&preamble_delay_list[0],0,64*sizeof(uint16_t));
  if (abstraction_flag == 0) {
    LOG_D(PHY,"[eNB %d][RAPROC] Frame %d, Subframe %d : PRACH RX Signal Power : %d dBm\n",phy_vars_eNB->Mod_id,
	  frame,subframe,dB_fixed(signal_energy(&phy_vars_eNB->lte_eNB_common_vars.rxdata[0][0][subframe*phy_vars_eNB->lte_frame_parms.samples_per_tti],512)) - phy_vars_eNB->rx_total_gain_eNB_dB);        
    
    //    LOG_I(PHY,"[eNB %d][RAPROC] PRACH: rootSequenceIndex %d, prach_ConfigIndex %d, zeroCorrelationZoneConfig %d, highSpeedFlag %d, prach_FreqOffset %d\n",phy_vars_eNB->Mod_id,phy_vars_eNB->lte_frame_parms.prach_config_common.rootSequenceIndex,phy_vars_eNB->lte_frame_parms.prach_config_common.prach_ConfigInfo.prach_ConfigIndex, phy_vars_eNB->lte_frame_parms.prach_config_common.prach_ConfigInfo.zeroCorrelationZoneConfig,phy_vars_eNB->lte_frame_parms.prach_config_common.prach_ConfigInfo.highSpeedFlag,phy_vars_eNB->lte_frame_parms.prach_config_common.prach_ConfigInfo.prach_FreqOffset);

    rx_prach(phy_vars_eNB,
	     subframe,
	     preamble_energy_list,
	     preamble_delay_list,
	     frame,
	     0);
  }
  else {
    for (UE_id=0;UE_id<NB_UE_INST;UE_id++) {
      
      LOG_D(PHY,"[RAPROC] UE_id %d (%p), generate_prach %d, UE RSI %d, eNB RSI %d preamble index %d\n",
	    UE_id,PHY_vars_UE_g[UE_id][CC_id],PHY_vars_UE_g[UE_id][CC_id]->generate_prach,
	    PHY_vars_UE_g[UE_id][CC_id]->lte_frame_parms.prach_config_common.rootSequenceIndex,
	    phy_vars_eNB->lte_frame_parms.prach_config_common.rootSequenceIndex,
	    PHY_vars_UE_g[UE_id][CC_id]->prach_PreambleIndex);
      
      if ((PHY_vars_UE_g[UE_id][CC_id]->generate_prach==1) &&
	  (PHY_vars_UE_g[UE_id][CC_id]->lte_frame_parms.prach_config_common.rootSequenceIndex ==
	   phy_vars_eNB->lte_frame_parms.prach_config_common.rootSequenceIndex) ) {
	preamble_energy_list[PHY_vars_UE_g[UE_id][CC_id]->prach_PreambleIndex] = 800;
	preamble_delay_list[PHY_vars_UE_g[UE_id][CC_id]->prach_PreambleIndex] = 5;
	
      }
    }
  }
  preamble_energy_max = preamble_energy_list[0];
  preamble_max = 0;

  for (i=1;i<64;i++) {
    if (preamble_energy_max < preamble_energy_list[i]) {
      preamble_energy_max = preamble_energy_list[i];
      preamble_max = i;
    }
  }
  	
    //printf("preamble_max=%d\n",preamble_max);
	//printf("preamble_energy_list=%d\n",preamble_energy_list[preamble_max]);
    
#ifdef DEBUG_PHY_PROC
  LOG_D(PHY,"[RAPROC] Most likely preamble %d, energy %d dB delay %d\n",
	preamble_max,
	preamble_energy_list[preamble_max],
	preamble_delay_list[preamble_max]);
#endif

  if (preamble_energy_list[preamble_max] > 750) {
    /*
    write_output("prach_ifft0.m","prach_t0",prach_ifft[0],2048,1,1);
    write_output("prach_rx0.m","prach_rx0",&phy_vars_eNB->lte_eNB_common_vars.rxdata[0][0][subframe*phy_vars_eNB->lte_frame_parms.samples_per_tti],6144+792,1,1);
    write_output("prach_rxF0.m","prach_rxF0",phy_vars_eNB->lte_eNB_prach_vars.rxsigF[0],24576,1,1);

    mac_xface->macphy_exit("Exiting for PRACH debug\n");
    */

    //UE_id = find_next_ue_index(phy_vars_eNB);//0916
  // if (UE_id>=0) {//0916
	if(1){
      //phy_vars_eNB->eNB_UE_stats[(uint32_t)UE_id].UE_timing_offset = preamble_delay_list[preamble_max]&0x1FFF; //limit to 13 (=11+2) bits
      //phy_vars_eNb->eNB_UE_stats[(uint32_t)UE_id].mode = PRACH;
      //phy_vars_eNB->eNB_UE_stats[(uint32_t)UE_id].sector = 0;
      LOG_I(PHY,"[eNB %d][RAPROC] Frame %d, subframe %d Initiating RA procedure with preamble %d, energy %d.%d dB, delay %d\n",
	    phy_vars_eNB->Mod_id,
	    frame,
	    subframe,
	    preamble_max,
	    preamble_energy_max/10,
	    preamble_energy_max%10,
	    preamble_delay_list[preamble_max]);
#ifdef OPENAIR2	  
    /*  mac_xface->initiate_ra_proc(phy_vars_eNB->Mod_id,
				  phy_vars_eNB->CC_id,
				  frame,
				  preamble_max,
				  preamble_delay_list[preamble_max],
				  0,subframe,0);*///20150915
		sRxDataInd[subframe].rachInd.numOfPdu=1;		  
		sRxDataInd[subframe].rachInd.rachdataInd[0].raRnti= 1+subframe+(10*0);
		sRxDataInd[subframe].rachInd.rachdataInd[0].preambleIndex= preamble_max;
		sRxDataInd[subframe].rachInd.rachdataInd[0].timingAdv= preamble_delay_list[preamble_max]/16;//timming advance for the preamble in units of 16Ts, Value 0 -> 1282
		sRxDataInd[subframe].rachInd.rachdataInd[0].rssi= 200;//0;//-127.5 ~ 0 dB, representing as 1275 ~ 0
  		sRxDataInd[subframe].rxFrameNum=frame;
		sRxDataInd[subframe].rxSubframeNum=subframe;
		RachPresentFlag[subframe]=1;
		//printf("[wewe]raRnti=%d\n",1+subframe+(10*0));
		printf("[wewe]preamble_max=%d\n",preamble_max);
		//printf("[wewe]preamble_delay=%d\n",preamble_delay_list[preamble_max]/16);
		//printf("[wewe]preamble_energy=%d\n",preamble_energy_list[preamble_max]);
		//printf("[wewe]rxFrameNum=%d\n",sRxDataInd[subframe].rxFrameNum);
		//printf("[wewe]rxSubframeNum=%d\n",sRxDataInd[subframe].rxSubframeNum);
        
	
		

#endif
    }
    else {
      LOG_I(PHY,"[eNB %d][RAPROC] frame %d, subframe %d: Unable to add user, max user count reached\n", 
	    phy_vars_eNB->Mod_id,frame, subframe);
    }
  }
}

void ulsch_decoding_procedures(unsigned char subframe, unsigned int i, PHY_VARS_eNB *phy_vars_eNB, unsigned char abstraction_flag)
{
  UNUSED(subframe);
  UNUSED(i);
  UNUSED(phy_vars_eNB);
  UNUSED(abstraction_flag);
  LOG_D(PHY,"ulsch_decoding_procedures not yet implemented. should not be called");
}

extern tMacaRxVectorReq sRxVectorReq[10];

#ifdef PRACHTHREAD // joe 20160517 create thread for PRACH
void phy_procedures_eNB_RX2(unsigned char sched_subframe,PHY_VARS_eNB *phy_vars_eNB,uint8_t abstraction_flag,relaying_type_t r_type)
{
	int subframe = phy_vars_eNB->proc[sched_subframe].subframe_rx;
	int frame = phy_vars_eNB->proc[sched_subframe].frame_rx;

	tMacaRxVectorReq	*pRxVector		 = NULL;
	pRxVector = &(sRxVectorReq[subframe]);
	tMacaTxVectorReq    *pTxVector       = NULL; 
	pTxVector = &(sTxVectorReq[subframe]); 
	if (sRxVectorReq[subframe].rachPresentFlag==1)
		{
		prach_procedures(phy_vars_eNB,sched_subframe,0);
		}
}
#endif

void phy_procedures_eNB_RX(unsigned char sched_subframe,PHY_VARS_eNB *phy_vars_eNB,uint8_t abstraction_flag,relaying_type_t r_type)
{
  RTIME time_start,time_end;
#ifdef MEASURERXTIME 
  time_start = rt_get_time_ns(); 
#endif
  //RX processing
  UNUSED(r_type);
  uint32_t l, ret=0,i,j,k;
  uint32_t sect_id=0;
  uint32_t harq_pid, harq_idx, round;
  uint8_t *pucch_payload=NULL,pucch_payload0[2]={0,0},pucch_payload1[2]={0,0};
  uint8_t SR_payload=0;
  int PUCCH_RSSI_POWER=0;
  int16_t n1_pucch0,n1_pucch1,n1_pucch2,n1_pucch3;
  uint8_t do_SR = 0;
  uint8_t pucch_sel = 0;
  int16_t metric0=0,metric1=0;
  ANFBmode_t bundling_flag;
  PUCCH_FMT_t format;
  uint8_t nPRS;
  //  uint8_t two_ues_connected = 0;
  uint8_t pusch_active = 0;
  LTE_DL_FRAME_PARMS *frame_parms=&phy_vars_eNB->lte_frame_parms;
  int sync_pos;
  uint16_t rnti=0;
  uint8_t access_mode;
  int num_active_cba_groups;
  int subframe = phy_vars_eNB->proc[sched_subframe].subframe_rx;
  int frame = phy_vars_eNB->proc[sched_subframe].frame_rx;
  int32_t ULSCH_rssi=0; //0916
  
  /********************ulsch FAPI Initial********************/
  //-----------------FAPI parameters---------------------
    HarqPresentFlag[subframe]=0;   //joe 20150904 initialize present flag
	CrcPresentFlag[subframe] =0;   //joe 20150904 initialize present flag
	RxPresentFlag[subframe]  =0;   //joe 20150904 initialize present flag
	SrPresentFlag[subframe]  =0;   //joe 20150904 initialize present flag

    tMacaRxDataInd    *pRxDataInd       = NULL; //wewe
	pRxDataInd = &(sRxDataInd[subframe]);//wewe
	tMacaRxVectorReq	*pRxVector		 = NULL;
	pRxVector = &(sRxVectorReq[subframe]);//wewe 20150526
	#if 1 //joe 20150812
	tMacaTxVectorReq    *pTxVector       = NULL; 
	pTxVector = &(sTxVectorReq[subframe]); 
	#endif
	memset(pRxDataInd,0,sizeof(tMacaRxDataInd));	
	uint8_t ulschPdu_total = 0;	 //total number of ULSCH PDU from RX vector 
	uint8_t ulschPdu_index = 0;   //index of ULSCH PDU from RX vector 
	uint8_t uciPdu_total = 0;     //total number of UCI PDU from RX vector   
	uint8_t uciPdu_index = 0 ;     //index of UCI PUD from RX vector 
	uint8_t cqi_index = 0;       //index of ULSCH for CQI PDU for indication
	uint8_t ulsch_index = 0;      //index of ULSCH for ULSCH indication for indication
	uint8_t sr_index = 0;
	uint8_t sr_total = 0;
	uint8_t sr_count = 0;
	uint8_t harq_index = 0;      //for harq indication
	uint8_t harq_total = 0;      //for number of harqs
	uint8_t hcch_t=0,cch_t =0,hsch_t=0,sch_t=0,hcch_c =0,cch_c =0,hsch_c=0,sch_c=0,break_index=0;//0916
	
	
	//tMacaRxVectorReq	*pRxVector		 = NULL;
	//pRxVector = &(sRxVectorReq[subframe]); //wewe 20150526
	tMacaUlschPduConfig *pUlschPduConfig = NULL;
	uint8_t PduType=0;

    tMacaCrcPduInd    *pCrcPduInd       = NULL;
    tMacaRbPowerInd   *pRbPowerInd      = NULL;
	tMacaUlschPduInd  *pUlschPduInd     = NULL;
    tMacaCqiPduInd    *pCqiPduInd       = NULL;

    //Jenny add numofCRC
	uint16 numofCRC=0;
	uint16 rxpdu_flag=0;
    int numofCRC_corret=0;	 
	int16_t Vr,Vi;  //by Jenny Vr, Vi vectors for TA estimation, by Jenny 20150601
	/*******************************************************/
	

  vcd_signal_dumper_dump_function_by_name(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_ENB_RX,1);
  start_meas(&phy_vars_eNB->phy_proc_rx);
#ifdef DEBUG_PHY_PROC
  LOG_D(PHY,"[eNB %d] Frame %d: Doing phy_procedures_eNB_RX(%d)\n",phy_vars_eNB->Mod_id,frame, subframe);
#endif

// ============== Remove 7.5 kHz rotation ==============
  if (abstraction_flag == 0) {
	  
    remove_7_5_kHz(phy_vars_eNB,subframe<<1);
    remove_7_5_kHz(phy_vars_eNB,(subframe<<1)+1);
	
  }
	  
// ====================  FFT =======================  
  if (abstraction_flag == 0) {
      
    for (l=0;l<phy_vars_eNB->lte_frame_parms.symbols_per_tti/2;l++) {
      slot_fep_ul(&phy_vars_eNB->lte_frame_parms,
		  &phy_vars_eNB->lte_eNB_common_vars,
		  l,
		  subframe<<1,
		  0,
		  0);
      slot_fep_ul(&phy_vars_eNB->lte_frame_parms,
		  &phy_vars_eNB->lte_eNB_common_vars,
		  l,
		  (subframe<<1)+1,
		  0,
		  0);
    }
   	 
  }
  sect_id = 0;



  // Check for active processes in current subframe
  harq_pid = subframe2harq_pid(&phy_vars_eNB->lte_frame_parms,frame,subframe);
  pusch_active = 0;
  
  // reset the cba flag used for collision detection 
  for (i=0; i < NUM_MAX_CBA_GROUP; i++){
    phy_vars_eNB->cba_last_reception[i]=0;
  }

 
//------------for TESTBENCH running-----------------------------------
  if (subframe>=start_subframe && force_ulsch_do_decode)
	  pRxVector->PduNum=1;

  
  for (i=0;i<(pRxVector->PduNum);i++)
  {
		 if(pRxVector->PduType[i]<=3)
			 ulschPdu_total++;
		 else
			 uciPdu_total++;			 		 
	 
	 
	 	if(pRxVector->PduType[i]==0||pRxVector->PduType[i]==2 || pRxVector->PduType[i]==4||pRxVector->PduType[i]==8){
		 pRxDataInd->numofPdu++;                    //pdu number for indication
		}else if(pRxVector->PduType[i]==1||pRxVector->PduType[i]==3){
		 pRxDataInd->numofPdu+=2;		             //pdu number for indication
		}		
		else{
		 //printf("wrong PDU nmber of pusch");	
		}
	 
		if(pRxVector->PduType[i]==5 || pRxVector->PduType[i]==7 )	 
	    sr_total++;
	 
	 if(pRxVector->PduType[i]==2 || pRxVector->PduType[i]==3 || pRxVector->PduType[i]==6 ||pRxVector->PduType[i]==7 ||pRxVector->PduType[i]==8 )	 
		harq_total++;
  }
	 
	 pRxDataInd->harqInd.numOfPdu = harq_total;
	 
	 
	   /* 
        Joe 20150916 Rxvector pdu reoder.
        The priority from high to low is Uci, UlschHarq, Ulsch, Srs.
    */
    # if REORDERPDU
    uint8_t W4[PHYUSER],numofUciHarqPdu=0;
    uint8_t W3[PHYUSER],numofUciPdu=0;
    uint8_t W2[PHYUSER],numofUlschPduHarqPdu=0;
    uint8_t W1[PHYUSER],numofUlschPdu=0;
    uint8_t W0[PHYUSER],numofSrsPdu=0;
    uint8_t RE_UCI[PHYUSER],RE_UCI_HARQ[PHYUSER],RE_ULSCH_HARQ[PHYUSER],RE_ULSCH[PHYUSER],_ReUci =0,_ReUlsch =0;
    uint8_t PduReOrder[PHYUSER*2];
    if ((pRxVector->PduNum) > PHYUSER)
        LOG_I(PHY,"Error in PDU reorder\n");
    for (i=0;i<PHYUSER;i++)
	{
		W4[i] = 0;
		W3[i] = 0;
		W2[i] = 0;
		W1[i] = 0;
		W0[i] = 0;
		RE_UCI[i] = 0;
		RE_UCI_HARQ[i] = 0;
		RE_ULSCH_HARQ[i] = 0;
		RE_ULSCH[i] = 0;
		PduReOrder[i] = 0;
	}	
    for(j=0;j<(pRxVector->PduNum);j++)
    {
		if ( (pRxVector->PduType[j] >= 4) && (pRxVector->PduType[j] <= 8) )
		{
			if ( (pRxVector->PduType[j] >= 6) && (pRxVector->PduType[j] <= 8) )
			{
				W4[numofUciHarqPdu] = j;
				RE_UCI_HARQ[numofUciHarqPdu] = _ReUci; 
				numofUciHarqPdu++;
			} 
			else if ( (pRxVector->PduType[j] == 4) || (pRxVector->PduType[j] == 5) )
			{
				W3[numofUciPdu] = j;
				RE_UCI[numofUciPdu] = _ReUci; 
				numofUciPdu++;
			}
			_ReUci ++;
		}
		else
		{
			if ( (pRxVector->PduType[j] == 2) || (pRxVector->PduType[j] == 3) )
			{
				W2[numofUlschPduHarqPdu] = j;
				RE_ULSCH_HARQ[numofUlschPduHarqPdu] = _ReUlsch;
				numofUlschPduHarqPdu++;
			}
			else if (pRxVector->PduType[j] == 9)
			{
				W0[numofSrsPdu] = j;
				numofSrsPdu++;
			}
			else 
			{
				W1[numofUlschPdu] = j;
				RE_ULSCH[numofUlschPdu] = _ReUlsch;
				numofUlschPdu++;
			}
			_ReUlsch ++;
		}
        
    }
	hcch_t =numofUciHarqPdu;
    cch_t =numofUciPdu;
	hsch_t=numofUlschPduHarqPdu;
	sch_t=numofUlschPdu;
	
    hcch_c =numofUciHarqPdu;
	cch_c =numofUciPdu;
	hsch_c=numofUlschPduHarqPdu;
	sch_c=numofUlschPdu;
	
    
    uint8_t i4index=0;
    uint8_t i3index=0;
    uint8_t i2index=0;
    uint8_t i1index=0;
    uint8_t i0index=0;
    for(i=0;i<(pRxVector->PduNum);i++)
    {
		if ( numofUciPdu > 0) 
        {
            PduReOrder[i] = W3[i3index];
            i3index++;
            numofUciPdu--;
        }   
        else if ( numofUciHarqPdu > 0) 
        {
            PduReOrder[i] = W4[i4index];
            i4index++;
            numofUciHarqPdu--;
        }  
        else if ( numofUlschPduHarqPdu > 0 )
        {
            PduReOrder[i] = W2[i2index];
            i2index++;
            numofUlschPduHarqPdu--;
        }
        else if ( numofUlschPdu > 0 )
        {
            PduReOrder[i] = W1[i1index];
            i1index++;
            numofUlschPdu--;
        }
        else if ( numofSrsPdu > 0 )
        {
            PduReOrder[i] = W0[i0index];
            i0index++;
            numofSrsPdu--;
        }
        //LOG_I(PHY,"PduReOrder[%d] = %d type = %d\n",i,PduReOrder[i],pRxVector->PduType[PduReOrder[i]]);
        
    }
    #endif
	 
	 
    if(hsch_t!=0)
      break_index=1; //0916 //joe 20150917
	 
	i4index=0;
    i3index=0;
    i2index=0;
    i1index=0;
    i0index=0;

    for (i=0;i<(pRxVector->PduNum);i++) { 

    //for (i=0;i<NUMBER_OF_UE_MAX;i++) { 


   // if (((phy_vars_eNB->ulsch_eNB[i]) &&
	//(phy_vars_eNB->ulsch_eNB[i]->rnti>0) &&
	//(phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->subframe_scheduling_flag==1)) || (pRxVector->PduType[0]<=3) { //wewe 20150526
    //Joe LOG_I(PHY,"[phy]rnti(%d) Pdutype(%d) uciidx(%d) i(%d) PduNum(%d)\n",pRxVector->uciConfig.uciPdu[uciIdx].ueId,pRxVector->PduType[i],uciIdx,i,pRxVector->PduNum);
   // if(pRxVector->PduType[i]<=3) {
	if((hcch_c==0)&&(cch_c==0)&&((hsch_c!=0)||(sch_c!=0))){	 //0916 //joe 20150918
      /*********************ulsch FAPI begin by Jenny***********************************************/
	  if(ulsch_FAPI_en){                //check location
	   //***RxVector
	   PduType=pRxVector->PduType[PduReOrder[i]];

	   if (( PduType == 2) || ( PduType == 3)) //20150921
	   {
		   ulschPdu_index = RE_ULSCH_HARQ[i2index];
		   i2index ++;
	   }
	   else if (( PduType == 0) || ( PduType == 1)) 
	   {
		   ulschPdu_index = RE_ULSCH[i1index];
		   i1index ++;
	   }
	   else
		   LOG_I(PHY," ERROR pdutype in RX ULSCH procedure\n");
	   


	   pUlschPduConfig = &(pRxVector->ulschConfig.ulschPdu[ulschPdu_index]);
	   //PDU Size
	   harq_pid=pUlschPduConfig->harqProcId;	
       phy_vars_eNB->proc[sched_subframe].harq_pid=pUlschPduConfig->harqProcId;	 
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->srs_active=pRxVector->srsConfigPresentFlag;
	 	 
	   //****ULSCH PDU  
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->TBS=pUlschPduConfig->tbSize;
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->rnti=pUlschPduConfig->rnti;  
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->first_rb = pUlschPduConfig->rbStart;
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->nb_rb= pUlschPduConfig->rbNum;
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->Qm=(pUlschPduConfig->modulation==0)?2:(pUlschPduConfig->modulation==1)?4:6;  
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->n_DMRS2=pUlschPduConfig->dmrs;
	   //Frequency enble flag
	   //Frequency Hopping bits
	   //New Data Indicator
	   //Redundancy version
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->rvidx=pUlschPduConfig->irv;
	   //UL Tx Mode
	   //Shorten_flag

	   //ULSCH_CQI_RI_ACK
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->O_RI=0;
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->O_ACK=0;
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->Or1=0;
	  
	
		if(PduType>0){
	     if(PduType==1||PduType==3){
			phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->O_RI=pUlschPduConfig->riBits;	  
			phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->Or1=pUlschPduConfig->cqiPmiBits;
			phy_vars_eNB->ulsch_eNB[ulschPdu_index]->beta_offset_cqi_times8 = beta_cqi[pUlschPduConfig->deltaOffsetCqi];//18;
			phy_vars_eNB->ulsch_eNB[ulschPdu_index]->beta_offset_ri_times8 = beta_ri[pUlschPduConfig->deltaOffsetRi];//10;
	     }
	     if(PduType==2||PduType==3)
		 {
			phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->O_ACK=pUlschPduConfig->harqBits;
			phy_vars_eNB->ulsch_eNB[ulschPdu_index]->beta_offset_harqack_times8= beta_ack[pUlschPduConfig->deltaOffsetHarq];//16;
			//HarqPresentFlag[subframe]=1;
			//pRxDataInd->harqInd.harqDataInd[harq_index].decodeFlag=0;
		 }
	    if(PduType==2||PduType==3)
			if(pUlschPduConfig->harqBits==0)
			{
				LOG_I(PHY,"pUlschPduConfig->harqBits=%d rnti(%d)\n",pUlschPduConfig->harqBits,pUlschPduConfig->rnti);
				LOG_I(PHY," PduNum(%d)  i(%d) Pdutype(%d)\n",pRxVector->PduNum,i,pRxVector->PduType[PduReOrder[i]]);
				LOG_I(PHY," hcch_t(%d)  cch_t(%d) hsch_t(%d) sch_t(%d)\n",hcch_t,cch_t,hsch_t,sch_t);
			}
		}
	   
       if(phy_vars_eNB->lte_frame_parms.frame_type==TDD)		
	     phy_vars_eNB->ulsch_eNB[ulschPdu_index]->bundling=(pUlschPduConfig->ackNackMode==0)?1:0;   //0601
	   else
	     phy_vars_eNB->ulsch_eNB[ulschPdu_index]->bundling=0;    //default:0  
	 
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->TBS=pUlschPduConfig->tbSize*8;
	   phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->Nsymb_pusch=12-(frame_parms->Ncp<<1)-(pRxVector->srsConfigPresentFlag==0?0:1);
       phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->Msc_initial   = 12*phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->nb_rb;
       phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->Nsymb_initial = phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->Nsymb_pusch;
	 
	/* 
	 LOG_I(PHY,"frame_rx(%d) subframe_rx(%d)\n",frame,subframe); //joe add debug 
	 LOG_I(PHY,"phy_vars_eNB->ulsch_eNB[%d]->harq_processes[%d]->srs_active=%d\n",i,harq_pid,phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->srs_active);
	 LOG_I(PHY,"phy_vars_eNB->ulsch_eNB[%d]->rnti=%d\n",i,phy_vars_eNB->ulsch_eNB[i]->rnti);
	 LOG_I(PHY,"phy_vars_eNB->ulsch_eNB[%d]->harq_processes[%d]->first_rb=%d\n",i,harq_pid,phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->first_rb );
	 LOG_I(PHY,"phy_vars_eNB->ulsch_eNB[%d]->harq_processes[%d]->n_DMRS2=%d\n",i,harq_pid,phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->n_DMRS2);
	 LOG_I(PHY,"phy_vars_eNB->ulsch_eNB[%d]->beta_offset_cqi_times8=%d\n",i,phy_vars_eNB->ulsch_eNB[i]->beta_offset_cqi_times8);
	 LOG_I(PHY,"phy_vars_eNB->ulsch_eNB[%d]->beta_offset_harqack_times8=%d\n",i,phy_vars_eNB->ulsch_eNB[i]->beta_offset_harqack_times8);
	 LOG_I(PHY,"phy_vars_eNB->ulsch_eNB[%d]->harq_processes[h%d]->O_ACK=%d\n",i,harq_pid,phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->O_ACK);
	 LOG_I(PHY,"phy_vars_eNB->ulsch_eNB[%d]->harq_processes[%d].Qm=%d\n",i,harq_pid,phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->Qm);  
	 */
	
	//ulschPdu_index++ ;
	}
	 //**************************************************************************** 
		

      pusch_active = 1;//20150915
      round = phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->round;



      nPRS = phy_vars_eNB->lte_frame_parms.pusch_config_common.ul_ReferenceSignalsPUSCH.nPRS[subframe<<1];

      phy_vars_eNB->ulsch_eNB[ulschPdu_index]->cyclicShift = (phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->n_DMRS2 + phy_vars_eNB->lte_frame_parms.pusch_config_common.ul_ReferenceSignalsPUSCH.cyclicShift + nPRS)%12;
       

#ifdef DEBUG_PHY_PROC
      LOG_D(PHY,"[eNB %d][PUSCH %d] Frame %d Subframe %d Demodulating PUSCH: dci_alloc %d, rar_alloc %d, round %d, first_rb %d, nb_rb %d, mcs %d, TBS %d, rv %d, cyclic_shift %d (n_DMRS2 %d, cyclicShift_common %d, nprs %d), O_ACK %d \n",
	    phy_vars_eNB->Mod_id,harq_pid,frame,subframe,
	    phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->dci_alloc,
	    phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->rar_alloc,
	    phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->round,
	    phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->first_rb,
	    phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->nb_rb,
	    phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->mcs,
	    phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->TBS,
	    phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->rvidx,
	    phy_vars_eNB->ulsch_eNB[i]->cyclicShift,
	    phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->n_DMRS2,
	    phy_vars_eNB->lte_frame_parms.pusch_config_common.ul_ReferenceSignalsPUSCH.cyclicShift,
	    nPRS,
	    phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->O_ACK);
#endif

	   	   if ((numofUlschPduHarqPdu) >= 2) //joe 20150918
		   LOG_I(PHY," Joe ulsch 123 PduNum(%d)  i(%d) Pdutype(%d)\n",pRxVector->PduNum,i,pRxVector->PduType[PduReOrder[i]]);
		   
		   
// ================= Channel estimation & LLR computation =================				      	  
      if (abstraction_flag==0) {
		 
	rx_ulsch(phy_vars_eNB,
		 sched_subframe,
							0,           // sector ID
		 ulschPdu_index,//i,
		 phy_vars_eNB->ulsch_eNB,
		 0,&Vr, &Vi);
							
      }

// =================== Decoding (CRC & ACK/NACK)==========================
      if (abstraction_flag == 0) {
		  
	ret = ulsch_decoding(phy_vars_eNB,
			     ulschPdu_index,//i,
			     sched_subframe,
			     0, // control_only_flag
			     phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->V_UL_DAI, 
			     0,
				 &harq_index,
										break_index);  
		
      }
	  
	    
		if(hsch_c==0)
			sch_c--;
		//if((hsch_c!=0)&&(hsch_t!=0))
		if(hsch_c!=0)
			hsch_c--; //0916

		if(break_index!=0)
		{
			//LOG_I(PHY,"continue.........\n");
			//continue;
			
			goto secndround;
			LOG_I(PHY,"continue success\n");
		}

      /*
	if ((two_ues_connected==1) && (phy_vars_eNB->cooperation_flag==2)) {

	}
	else {

*/
      //compute the expected ULSCH RX power (for the stats)
      phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->delta_TF =
	get_hundred_times_delta_IF_eNB(phy_vars_eNB,ulschPdu_index,harq_pid, 0); // 0 means bw_factor is not considered

      phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->subframe_scheduling_flag=0;

      if (ret == (1+MAX_TURBO_ITERATIONS)) {

      }  // ulsch in error
      else {
	LOG_D(PHY,"[eNB %d][PUSCH %d] Frame %d subframe %d ULSCH received, setting round to 0, PHICH ACK\n",
	      phy_vars_eNB->Mod_id,harq_pid,
	      frame,subframe);	    


	if (phy_vars_eNB->ulsch_eNB[i]->Msg3_flag == 1) {

	}
	else {
	  
#ifdef OPENAIR2
          

#endif
	}
	

      }  // ulsch not in error
      
      

        /*============================Indication for FAPI, by Jenny===================================*/
   	    /*************add RX indication**************/
		
		pUlschPduInd = &(pRxDataInd->ulschInd.ulschDataInd[ulsch_index]);
		pCqiPduInd   = &(pRxDataInd->cqiInd.cqiDataInd[cqi_index]);
		//pCrcPduInd   = &(pRxDataInd->crcInd._crcInd[ulsch_index]); 
		pCrcPduInd   = &(pRxDataInd->crcInd._crcInd[ulschPdu_index]); //joe 20160425 modify crc index 
		// 20150810 joe add rssi
	    ULSCH_rssi=dB_fixed(phy_vars_eNB->lte_eNB_pusch_vars[ulschPdu_index]->ulsch_power[0]*
		     (phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->nb_rb*12)/
		     phy_vars_eNB->lte_frame_parms.ofdm_symbol_size) -
	    phy_vars_eNB->rx_total_gain_eNB_dB -
	    hundred_times_log10_NPRB[phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->nb_rb-1]/100 -
	    get_hundred_times_delta_IF_eNB(phy_vars_eNB,ulschPdu_index,harq_pid, 0)/100;
		
		//LOG_I(PHY,"RSSI per RB=%d, DMRS power=%d\n",phy_vars_eNB->eNB_UE_stats[i].UL_rssi[0],dB_fixed(phy_vars_eNB->lte_eNB_pusch_vars[i]->ulsch_power[0]));	
		
		pCrcPduInd->flag=0; //joe 20150812		
		
		#if 1 //joe 20150812
		/********** CRC Indication**************/ 
		  pCrcPduInd->rnti=phy_vars_eNB->ulsch_eNB[ulschPdu_index]->rnti;  
		  numofCRC++;
		  pRxDataInd->crcInd.numOfCrcPdu=numofCRC;		    
	      
		  pCrcPduInd->flag=(ret==(1+MAX_TURBO_ITERATIONS))?1:0;
	  	  numofCRC_corret=numofCRC_corret+1;			
	
		  pCrcPduInd->rssi=ULSCH_rssi*2+255; // 20150810 joe add rssi
		  //LOG_I(PHY,"FAPI RSSI per RB=%d\n",pCrcPduInd->rssi);
		  pCrcPduInd->effectiveSinr=255;
		  if(pCrcPduInd->flag!=0)
		  {
			LOG_I(PHY,"(%d,%d),CRC(%d),harqpid(%d),power(%d),N0(%d),RNTI(%d)\n",pTxVector->txFrameNum,subframe
				,pCrcPduInd->flag
				,harq_pid,
				dB_fixed(phy_vars_eNB->lte_eNB_pusch_vars[ulschPdu_index]->ulsch_power[0])
				,phy_vars_eNB->PHY_measurements_eNB->n0_power_dB[0]
				,phy_vars_eNB->ulsch_eNB[ulschPdu_index]->rnti
				);	
				
	/*	   LOG_I(PHY,"[eNB %d][PUSCH %d] (%d/%d)Demodulating PUSCH:  round %d, first_rb %d, nb_rb %d, mcs %d, TBS %d, rv %d, cyclic_shift %d (n_DMRS2 %d, cyclicShift_common %d, nprs %d), O_ACK %d \n",
				phy_vars_eNB->Mod_id,harq_pid,subframe,frame,
				phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->round,
				phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->first_rb,
				phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->nb_rb,
				phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->mcs,
				phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->TBS,
				phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->rvidx,
				phy_vars_eNB->ulsch_eNB[i]->cyclicShift,
				phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->n_DMRS2,
				phy_vars_eNB->lte_frame_parms.pusch_config_common.ul_ReferenceSignalsPUSCH.cyclicShift,
				nPRS,
				phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->O_ACK);*/
		  }
		  
		/*************************************/ 
		  #endif
		if (ret<(1+MAX_TURBO_ITERATIONS)){       //transmit pusch pdu when CRC correct
		  //transmit data 
		  memcpy((pUlschPduInd->Tb),
	      phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->b,
	      phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->TBS/8);
		  //***write output***//
		/*  for (int b=0;b<phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->TBS/8;b++)
		  printf("decoding out=%x,%x\n",phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->b[b],pUlschPduInd->Tb[b]);
		*/
		  /*****for ULSCH PDU****************/
 		  pUlschPduInd->rnti=phy_vars_eNB->ulsch_eNB[ulschPdu_index]->rnti;
		  pUlschPduInd->tbSize=phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->TBS/8;
		//pUlschPduInd->dataOffset=1;            //?	
          pUlschPduInd->pduType = 0;
          pUlschPduInd->ri =phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->o_RI[0];
          pUlschPduInd->timingAdvanceReal = Vr;
          pUlschPduInd->timingAdvanceImage = Vi;	

		  /**********************************/
        		
		  #if 0 // joe 20150812  
		  /********** CRC Indication**************/ 
	   
		  pCrcPduInd->rnti=phy_vars_eNB->ulsch_eNB[ulschPdu_index]->rnti;  
		  numofCRC++;
		  pRxDataInd->crcInd.numOfCrcPdu=numofCRC;		    
	      pCrcPduInd->flag=0;//(ret==(1+MAX_TURBO_ITERATIONS))?1:0;  //1 for error, 0 for correct
	  	  numofCRC_corret=numofCRC_corret+1;			
	      //if(pCrcPduInd->flag==1)
		  //printf("frame=%d, subframe=%d, CRC ind=%d, UL_power=%d, ret=%d\n",frame,subframe,pCrcPduInd->flag,dB_fixed(phy_vars_eNB->lte_eNB_pusch_vars[i]->ulsch_power[0]),ret);
	    //  LOG_I(PHY,"frame=%d, subframe=%d, CRC ind=%d, UL_power=%d, N0=%d\n",frame,subframe,pCrcPduInd->flag,dB_fixed(phy_vars_eNB->lte_eNB_pusch_vars[i]->ulsch_power[0]),phy_vars_eNB->PHY_measurements_eNB->n0_power_dB[0]);			
		  pCrcPduInd->rssi=ULSCH_rssi*2+255; // 20150810 joe add rssi
		  //pCrcPduInd->rssi=255;
		  //LOG_I(PHY,"FAPI RSSI per RB=%d\n",pCrcPduInd->rssi);
		  pCrcPduInd->effectiveSinr=255;
		  /*************************************/
		  #endif
		  ulsch_index++;	
		  rxpdu_flag++;

		}
		else{
			pRxDataInd->numofPdu--;	
		}	
		if(ulschPdu_index==(ulschPdu_total-1) && numofCRC>0 ){
			CrcPresentFlag[subframe]=1;    //flag required by Joe
		}
		

		/*******for CQI PDU*****************/      
		if(pRxVector->PduType[PduReOrder[i]]==1 || pRxVector->PduType[PduReOrder[i]]==3){
		if (phy_vars_eNB->ulsch_eNB[ulschPdu_index]->harq_processes[harq_pid]->cqi_crc_status == 1){
        pCqiPduInd->rnti =phy_vars_eNB->ulsch_eNB[ulschPdu_index]->rnti; //same rnti?
		//pCqiPduInd->pduLength =0;
		pCqiPduInd->pduLength =20;		 	    
       // pCqiPduInd->dataOffset = 1;        //? no need, will be calculated by FAPI  
        pCqiPduInd->pduType =1;
        pCqiPduInd->ri = 0;                //?
        pCqiPduInd->timingAdvanceReal =1;
        pCqiPduInd->timingAdvanceImage = 0;
        pCqiPduInd->cqi = phy_vars_eNB->ulsch_eNB[i]->harq_processes[harq_pid]->o;  //?
        pCqiPduInd->pucchRssi =200;  
		cqi_index++;
		rxpdu_flag++;
		}
		else{
			pRxDataInd->numofPdu--;
		}
		}
/*===========================Indication End, Jenny=========================*/
	    ulschPdu_index++;
		   
    }

#if 1
   // else if ((phy_vars_eNB->dlsch_eNB[i][0]) &&
	//     (phy_vars_eNB->dlsch_eNB[i][0]->rnti>0)||(pRxVector->PduType[0]>=4)) { // check for PUCCH
  //else if ((pRxVector->PduType[i]>=4)) { // check for PUCCH
  else if ((hcch_c!=0) || (cch_c!=0)) { // check for PUCCH //joe 20150917
		 
      // check SR availability
      //do_SR = is_SR_subframe(phy_vars_eNB,i,sched_subframe); //wewe
      //      do_SR = 0;
    
      // Now ACK/NAK
      // First check subframe_tx flag for earlier subframes
	  PduType=pRxVector->PduType[PduReOrder[i]]; //20150921
	  n1_pucch0=-1;
	  n1_pucch1=-1;
	   if (( PduType == 6) || ( PduType == 7) || ( PduType == 8)) 
	   {
		   uciPdu_index = RE_UCI_HARQ[i4index];
		   i4index ++;
	   }
	   else if (( PduType == 4) || ( PduType == 5)) 
	   {
		   uciPdu_index = RE_UCI[i3index];
		   i3index ++;
	   }
	   else
		   LOG_I(PHY," ERROR pdutype in RX UCI procedure\n");
 switch (pRxVector->PduType[PduReOrder[i]]) // joe 20150917
	  {
		  case 4:
		  format=pucch_format2;	  
		  n1_pucch0=pRxVector->uciConfig.uciPdu[uciPdu_index].resourceIndexExtra;	  
		  break;
		  
		  case 5:
		  //UCI_SR
		  format=pucch_format1;
		  do_SR=1;
		  break;
		  
		  case 6:
		  // UCI_HARQ
		  format=pucch_format1a;
		  if(pRxVector->uciConfig.uciPdu[uciPdu_index].harqBits==2)
		  format=pucch_format1b;
		  n1_pucch0=pRxVector->uciConfig.uciPdu[uciPdu_index].resourceIndexExtra;	  
		  break;
		  
		  case 7:		 
		  do_SR=1;		  
		  format=pucch_format1;
		  n1_pucch0=pRxVector->uciConfig.uciPdu[uciPdu_index].resourceIndexExtra;
		  break;
		  
		  case 8:
		    format=pucch_format2a;
		    if(pRxVector->uciConfig.uciPdu[uciPdu_index].harqBits==2)
				 format=pucch_format2b;
		  n1_pucch0=pRxVector->uciConfig.uciPdu[uciPdu_index].resourceIndexExtra;
		  break;
		  
	  }
	  
	  
      if ((n1_pucch0==-1) && (n1_pucch1==-1) && (do_SR==0)) {  // no TX PDSCH that have to be checked and no SR for this UE_id
	  
      }
      else {
		  
	// otherwise we have some PUCCH detection to do
      
	if ((pRxVector->PduType[PduReOrder[i]] == 5)||(pRxVector->PduType[PduReOrder[i]] == 7)) { //wewe20150526
	
	 sr_count++;
	
	  if (abstraction_flag == 0)
	    metric0 = rx_pucch(phy_vars_eNB,
			       format,
			       i,
			      // phy_vars_eNB->scheduling_request_config[i].sr_PUCCH_ResourceIndex,				 					
				   pRxVector->uciConfig.uciPdu[uciPdu_index].resourceIndex, //wewe 20150526
			       0, // n2_pucch
			       1, // shortened format
			       &SR_payload,
			       subframe,
			       PUCCH1_THRES,
										&PUCCH_RSSI_POWER);

	  if (SR_payload > 0) {
	  

		pRxDataInd->srInd.srDataInd[sr_index].pucchRssi=PUCCH_RSSI_POWER;
		pRxDataInd->srInd.srDataInd[sr_index].rnti=pRxVector->uciConfig.uciPdu[uciPdu_index].ueId;
		pRxDataInd->srInd.numOfPdu=sr_index+1;

		sr_index++;
	  } 
	  
	  
	  if(sr_count==sr_total)
	   SrPresentFlag[subframe]=1;
	  
	}// do_SR==1
	

	if ((pRxVector->PduType[PduReOrder[i]] == 6)||(pRxVector->PduType[PduReOrder[i]] == 7)||(pRxVector->PduType[PduReOrder[i]] == 8)) { // FDD
	  
	  // if SR was detected, use the n1_pucch from SR, else use n1_pucch0
	  //HarqPresentFlag[subframe]=1;
	  	//pRxDataInd->harqInd.harqDataInd[harq_index].decodeFlag=1;
		//pRxDataInd->harqInd.harqDataInd[harq_index].rnti=pRxVector->uciConfig.uciPdu[uciPdu_index].ueId;
	//	LOG_I(PHY,"harqBits(%d)\n",pRxVector->uciConfig.uciPdu[uciPdu_index].harqBits);
	if (pRxVector->PduType[PduReOrder[i]] == 7) //joe 20150917
	{  
	format=pucch_format1a;
    if(pRxVector->uciConfig.uciPdu[uciPdu_index].harqBits==2)
	format=pucch_format1b;
	 n1_pucch0 = (SR_payload==1) ? pRxVector->uciConfig.uciPdu[uciPdu_index].resourceIndex:n1_pucch0;//wewe 20150526
	}
	
	if (abstraction_flag == 0)
	{
	    metric0 = rx_pucch(phy_vars_eNB,
			       format,
			       i,
			       (uint16_t)n1_pucch0,
			       0, //n2_pucch
			       1, // shortened format
			       pucch_payload0,
			       subframe,
			       PUCCH1a_THRES,
				   &PUCCH_RSSI_POWER);
	  }	

		//pRxDataInd->harqInd.numOfPdu = 1; // joe add number of harq 	
		pRxDataInd->harqInd.harqDataInd[harq_index].rnti=pRxVector->uciConfig.uciPdu[uciPdu_index].ueId;
		// LOG_I(PHY,"rnti(%d)\n",pRxVector->uciConfig.uciPdu[uciPdu_index].ueId);
		pRxDataInd->harqInd.harqDataInd[harq_index].harqTb1=(pucch_payload0[0]>0)?1:2;
		pRxDataInd->harqInd.harqDataInd[harq_index].harqTb2=(pucch_payload0[1]>0)?1:2;
		pRxDataInd->harqInd.harqDataInd[harq_index].pucchRssi=PUCCH_RSSI_POWER;
		//LOG_I(PHY,"PUCCH_RSSI_POWER=%d\n\n",PUCCH_RSSI_POWER);
		if(format==pucch_format1a)
		if ((pRxDataInd->harqInd.harqDataInd[harq_index].harqTb1!=1 ))
	    LOG_I(PHY,"PUCCH 1a NACK \n");
	//   HarqPresentFlag[subframe]=1;
		if ((pRxDataInd->harqInd.harqDataInd[harq_index].rnti == 0 ))
		LOG_I(PHY,"error PUCCH rnti 0 \n");
		
		if(format==pucch_format1b)
		if ((pRxDataInd->harqInd.harqDataInd[harq_index].harqTb1!=1 )||(pRxDataInd->harqInd.harqDataInd[harq_index].harqTb2!=1 ))
	    {
		LOG_I(PHY,"PUCCH 1b NACK \n");
		LOG_I(PHY,"TB1 %d, TB2 %d\n",pucch_payload0[0],pucch_payload0[1]);
		}
		
		harq_index++;
	    
	} // FDD
	
	if(phy_vars_eNB->lte_frame_parms.frame_type==TDD) {  //TDD
	
	  bundling_flag = phy_vars_eNB->pucch_config_dedicated[i].tdd_AckNackFeedbackMode;
	
	  // fix later for 2 TB case and format1b

	  if ((frame_parms->frame_type==FDD) || 
	      (bundling_flag==bundling)    || 
	      ((frame_parms->frame_type==TDD)&&(frame_parms->tdd_config==1)&&((subframe!=2)||(subframe!=7)))) {
	    format = pucch_format1a;
	    //	    msg("PUCCH 1a\n");
	  }
	  else {
	    format = pucch_format1b;
	    //	    msg("PUCCH 1b\n");
	  }
	
	  // if SR was detected, use the n1_pucch from SR
	  if (SR_payload==1) {
	    if (abstraction_flag == 0) 
	      metric0 = rx_pucch(phy_vars_eNB,
				 format,
				 i,
				 phy_vars_eNB->scheduling_request_config[i].sr_PUCCH_ResourceIndex,
				 0, //n2_pucch
				 1, // shortened format
				 pucch_payload0,
				 subframe,
				 PUCCH1a_THRES,
				   &PUCCH_RSSI_POWER//wewe0814
				 );
	  }
	  else {  //using n1_pucch0/n1_pucch1 resources

	    metric0=0;
	    metric1=0;
	    
	    // Check n1_pucch0 metric
	    if (n1_pucch0 != -1) {
	      if (abstraction_flag == 0) 
		metric0 = rx_pucch(phy_vars_eNB,
				   format,
				   i,
				   (uint16_t)n1_pucch0,
				   0, // n2_pucch
				   1, // shortened format
				   pucch_payload0,
				   subframe,
				   PUCCH1a_THRES,
				   &PUCCH_RSSI_POWER//wewe0814
				   );
	    }

	    // Check n1_pucch1 metric
	    if (n1_pucch1 != -1) {
	      if (abstraction_flag == 0)
		metric1 = rx_pucch(phy_vars_eNB,
				   format,
				   i,
				   (uint16_t)n1_pucch1,
				   0, //n2_pucch
				   1, // shortened format
				   pucch_payload1,
				   subframe,
				   PUCCH1a_THRES,
				   &PUCCH_RSSI_POWER//wewe0814
				   );
	    }
	  }

	  if (SR_payload == 1) {
	    pucch_payload = pucch_payload0;
	    if (bundling_flag == bundling)
	      pucch_sel = 2;
	  }
	  else if (bundling_flag == multiplexing) {  // multiplexing + no SR
	    pucch_payload = (metric1>metric0) ? pucch_payload1 : pucch_payload0;
	    pucch_sel     = (metric1>metric0) ? 1 : 0;
	  }
	  else { // bundling + no SR
	    if (n1_pucch1 != -1)
	      pucch_payload = pucch_payload1;
	    else if (n1_pucch0 != -1)
	      pucch_payload = pucch_payload0;
	    pucch_sel = 2;  // indicate that this is a bundled ACK/NAK  
	  }

	  process_HARQ_feedback(i,sched_subframe,phy_vars_eNB,
				0,// pusch_flag
				pucch_payload,
				pucch_sel,
				SR_payload);
	}
      }
	  uciPdu_index++;
	  
	  
		if ((pRxVector->PduType[PduReOrder[i]] == 6)||(pRxVector->PduType[PduReOrder[i]] == 7)||(pRxVector->PduType[PduReOrder[i]] == 8))
			hcch_c--;// joe 20150917
		else if ((pRxVector->PduType[PduReOrder[i]] == 4)||(pRxVector->PduType[PduReOrder[i]] == 5))
			cch_c--;//joe 20150917
			pusch_active = 1; //joe 20160415
    } // PUCCH processing
    
#endif //PUCCH
  
  secndround:
  
	if((i==((cch_t+hcch_t+hsch_t)-1))&&(break_index==1)) // joe 20150917 for the harq which only in pucch 
    {
        i-=hsch_t;
		i2index-=hsch_t; //joe 20150921
        break_index=0;
        HarqPresentFlag[subframe]=1;
        hsch_c+=hsch_t;
        //ulschPdu_index-=hsch_t; //20150921
		//LOG_I(PHY,"frame=%d, subframe=%d, hsch_t=%d, hsch_c=%d, cch_t=%d hcch_t=%d\n",frame,subframe,hsch_t,hsch_c,cch_t,hcch_t);
    }
    else if( (i==((cch_t+hcch_t)-1)) && (hsch_t == 0)&&(hcch_t!=0)) // joe 20150917 for the harq which only in pucch 
    {
        HarqPresentFlag[subframe]=1;
		//LOG_I(PHY,"CC frame=%d, subframe=%d, hsch_t=%d, hsch_c=%d, cch_t=%d hcch_t=%d\n",frame,subframe,hsch_t,hsch_c,cch_t,hcch_t);
    }

  
  //if((i==pRxVector->PduNum-1)&& numofCRC_corret>0 )                  //test, i should be max user //0608
  if((i==pRxVector->PduNum-1) && (rxpdu_flag!=0))
  {
	RxPresentFlag[subframe] = 1;
  	pRxDataInd->numofulsch=ulsch_index;
    pRxDataInd->numofcqi=cqi_index;
	
	if ((pRxDataInd->numofPdu) == 0)
		printf(" RxDataInd->numofPdu == 0 rxpresent is enabled\n");
  }
  
   //if((i==pRxVector->PduNum-1) && (harq_total==harq_index) && (harq_total!=0))
   if((harq_total==harq_index) && (harq_total!=0))//wewe 20150720
   {
	
    // LOG_I(PHY," weweweweweweweweewewe HARQ  \n");
	    
    }
      
  } // loop i=0 ... NUMBER_OF_UE_MAX-1

#ifndef PRACHTHREAD  // joe 20160517 create thread for PRACH
/*
  if(subframe==0)
  {
    prach_counter++;
  }
  if(prach_counter>5000)
  {
    sRxVectorReq[subframe].rachPresentFlag=0;
  }
  else if(prach_counter>5001)
  	prach_counter--;
  else if(prach_counter==4999)
  	LOG_E(PHY,"PRACH disable\n");
  */
  
  if (sRxVectorReq[subframe].rachPresentFlag==1){
    vcd_signal_dumper_dump_function_by_name(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_ENB_PRACH_RX,1);	
    prach_procedures(phy_vars_eNB,sched_subframe,abstraction_flag);	
    vcd_signal_dumper_dump_function_by_name(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_ENB_PRACH_RX,0);
	pusch_active = 1; //joe 20160415
  }
#endif

  if (pusch_active == 0){
    if (abstraction_flag == 0) {

      lte_eNB_I0_measurements(phy_vars_eNB,
			      0,
			      phy_vars_eNB->first_run_I0_measurements);
    }
    
    if (I0_clear == 1)
      I0_clear = 0;
  }

#ifdef EMOS
  phy_procedures_emos_eNB_RX(subframe,phy_vars_eNB);
#endif

  vcd_signal_dumper_dump_function_by_name(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_ENB_RX,0);

#ifdef MEASURERXTIME
  time_end = rt_get_time_ns();
  if((time_end-time_start>=1200000) && (time_end-time_start<=4000000) )
    LOG_I(PHY,"(%d,%d)PHY RX process time exceed 1.2ms, time(%llu)\n",pTxVector->txFrameNum,subframe,time_end-time_start);
#endif
   
}

#undef DEBUG_PHY_PROC

#ifdef Rel10  
int phy_procedures_RN_eNB_TX(unsigned char last_slot, unsigned char next_slot, relaying_type_t r_type){
  
  int do_proc=0;// do nothing
  switch(r_type){
  case no_relay:
    do_proc= no_relay; // perform the normal eNB operation 
    break;
  case multicast_relay:
    if (((next_slot >>1) < 6) || ((next_slot >>1) > 8))
      do_proc = 0; // do nothing
    else // SF#6, SF#7 and SF#8 
      do_proc = multicast_relay; // do PHY procedures eNB TX 
    break;
  default: // should'not be here
    LOG_W(PHY,"Not supported relay type %d, do nothing\n", r_type);
    do_proc=0; 
    break;
  }
  return do_proc;
}
#endif 
void phy_procedures_eNB_lte(unsigned char subframe,PHY_VARS_eNB **phy_vars_eNB,uint8_t abstraction_flag, 
			    relaying_type_t r_type, PHY_VARS_RN *phy_vars_rn) {
#if defined(ENABLE_ITTI)
  MessageDef   *msg_p;
  const char   *msg_name;
  instance_t    instance;
  unsigned int  Mod_id;
  int           result;
#endif


  int CC_id=0;

  /*
    if (phy_vars_eNB->proc[sched_subframe].frame_tx >= 1000)
    mac_xface->macphy_exit("Exiting after 1000 Frames\n");
  */
  vcd_signal_dumper_dump_variable_by_name(VCD_SIGNAL_DUMPER_VARIABLES_FRAME_NUMBER_TX_ENB, phy_vars_eNB[0]->proc[subframe].frame_tx);
  vcd_signal_dumper_dump_function_by_name(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_ENB_LTE,1);
  start_meas(&phy_vars_eNB[0]->phy_proc);

#if defined(ENABLE_ITTI)
  do {
    // Checks if a message has been sent to PHY sub-task
    itti_poll_msg (TASK_PHY_ENB, &msg_p);

    if (msg_p != NULL) {
      msg_name = ITTI_MSG_NAME (msg_p);
      instance = ITTI_MSG_INSTANCE (msg_p);
      Mod_id = instance;

      switch (ITTI_MSG_ID(msg_p)) {
#   if defined(ENABLE_RAL)
      case TIMER_HAS_EXPIRED:
	// check if it is a measurement timer
	{
	  hashtable_rc_t       hashtable_rc;

	  hashtable_rc = hashtable_is_key_exists(PHY_vars_eNB_g[Mod_id][0]->ral_thresholds_timed, (uint64_t)(TIMER_HAS_EXPIRED(msg_p).timer_id));
	  if (hashtable_rc == HASH_TABLE_OK) {
	    phy_eNB_lte_check_measurement_thresholds(instance, (ral_threshold_phy_t*)TIMER_HAS_EXPIRED(msg_p).arg);
	  }
	}
	break;


      case PHY_MEAS_THRESHOLD_REQ:
#warning "TO DO LIST OF THRESHOLDS"
	LOG_D(PHY, "[ENB %d] Received %s\n", Mod_id, msg_name);
	{
	  ral_threshold_phy_t* threshold_phy_p  = NULL;
	  int                  index, res;
	  long                 timer_id;
	  hashtable_rc_t       hashtable_rc;

	  switch (PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param.th_action) {

	  case RAL_TH_ACTION_CANCEL_THRESHOLD:
	    break;

	  case RAL_TH_ACTION_SET_NORMAL_THRESHOLD:
	  case RAL_TH_ACTION_SET_ONE_SHOT_THRESHOLD:
	    for (index = 0; index < PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param.num_thresholds; index++) {
	      threshold_phy_p                  = calloc(1, sizeof(ral_threshold_phy_t));
	      threshold_phy_p->th_action       = PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param.th_action;
	      memcpy(&threshold_phy_p->link_param.link_param_type,
		     &PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param.link_param_type,
		     sizeof(ral_link_param_type_t));

	      memcpy(&threshold_phy_p->threshold,
		     &PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param.thresholds[index],
		     sizeof(ral_threshold_t));

	      switch (PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param.union_choice) {

	      case RAL_LINK_CFG_PARAM_CHOICE_TIMER_NULL:
		switch (PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param.link_param_type.choice) {
		case RAL_LINK_PARAM_TYPE_CHOICE_GEN:
		  SLIST_INSERT_HEAD(
				    &PHY_vars_eNB_g[Mod_id][0]->ral_thresholds_gen_polled[PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param.link_param_type._union.link_param_gen],
				    threshold_phy_p,
				    ral_thresholds);
		  break;

		case RAL_LINK_PARAM_TYPE_CHOICE_LTE:
		  SLIST_INSERT_HEAD(
				    &PHY_vars_eNB_g[Mod_id][0]->ral_thresholds_lte_polled[PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param.link_param_type._union.link_param_lte],
				    threshold_phy_p,
				    ral_thresholds);
		  break;

		default:
		  LOG_E(PHY, "[ENB %d] BAD PARAMETER cfg_param.link_param_type.choice %d in %s\n",
			Mod_id, PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param.link_param_type.choice, msg_name);
		}
		break;

	      case RAL_LINK_CFG_PARAM_CHOICE_TIMER:
		res = timer_setup(
				  (uint32_t)(PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param._union.timer_interval/1000),//uint32_t      interval_sec,
				  (uint32_t)(PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param._union.timer_interval%1000),//uint32_t      interval_us,
				  TASK_PHY_ENB,
				  instance,
				  TIMER_PERIODIC,
				  threshold_phy_p,
				  &timer_id);

		if (res == 0) {
		  hashtable_rc = hashtable_insert(PHY_vars_eNB_g[Mod_id][0]->ral_thresholds_timed, (uint64_t )timer_id, (void*)threshold_phy_p);
		  if (hashtable_rc == HASH_TABLE_OK) {
		    threshold_phy_p->timer_id = timer_id;
		  } else {
		    LOG_E(PHY, "[ENB %d]  %s: Error in hashtable. Could not configure threshold index %d \n",
			  Mod_id, msg_name, index);
		  }

		} else {
		  LOG_E(PHY, "[ENB %d]  %s: Could not configure threshold index %d because of timer initialization failure\n",
			Mod_id, msg_name, index);
		}
		break;

	      default: // already checked in RRC, should not happen here
		LOG_E(PHY, "[ENB %d] BAD PARAMETER cfg_param.union_choice %d in %s\n",
		      Mod_id, PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param.union_choice, msg_name);
	      }
	    }
	    break;

	  default:
	    LOG_E(PHY, "[ENB %d] BAD PARAMETER th_action value %d in %s\n",
		  Mod_id, PHY_MEAS_THRESHOLD_REQ(msg_p).cfg_param.th_action, msg_name);
	  }

	}
	break;
#   endif

	/* Messages from eNB app */
      case PHY_CONFIGURATION_REQ:
	LOG_I(PHY, "[eNB %d] Received %s\n", instance, msg_name);
	/* TODO */

	break;

      default:
	LOG_E(PHY, "[ENB %d] Received unexpected message %s\n", Mod_id, msg_name);
	break;
      }

      result = itti_free (ITTI_MSG_ORIGIN_ID(msg_p), msg_p);
      AssertFatal (result == EXIT_SUCCESS, "Failed to free memory (%d)!\n", result);
    }
  } while(msg_p != NULL);
#endif


  for (CC_id=0;CC_id<MAX_NUM_CCs; CC_id++) {
    if ((((phy_vars_eNB[CC_id]->lte_frame_parms.frame_type == TDD)&&
	  (subframe_select(&phy_vars_eNB[CC_id]->lte_frame_parms,phy_vars_eNB[CC_id]->proc[subframe].subframe_tx)==SF_DL))||
	 (phy_vars_eNB[CC_id]->lte_frame_parms.frame_type == FDD))) {
#ifdef Rel10 
      if (phy_procedures_RN_eNB_TX(phy_vars_eNB[CC_id]->proc[subframe].subframe_rx, phy_vars_eNB[CC_id]->proc[subframe].subframe_tx, r_type) != 0 ) 
#endif 
	phy_procedures_eNB_TX(subframe,phy_vars_eNB[CC_id],abstraction_flag,r_type,phy_vars_rn);
    }
    if ((((phy_vars_eNB[CC_id]->lte_frame_parms.frame_type == TDD )&&
	  (subframe_select(&phy_vars_eNB[CC_id]->lte_frame_parms,phy_vars_eNB[CC_id]->proc[subframe].subframe_rx)==SF_UL)) ||
	 (phy_vars_eNB[CC_id]->lte_frame_parms.frame_type == FDD))){
      phy_procedures_eNB_RX(subframe,phy_vars_eNB[CC_id],abstraction_flag,r_type);
    }
    if (subframe_select(&phy_vars_eNB[CC_id]->lte_frame_parms,phy_vars_eNB[CC_id]->proc[subframe].subframe_tx)==SF_S) {
#ifdef Rel10 
      if (phy_procedures_RN_eNB_TX(subframe, subframe, r_type) != 0 )
#endif 
	phy_procedures_eNB_TX(subframe,phy_vars_eNB[CC_id],abstraction_flag,r_type,phy_vars_rn);
    }
    if ((subframe_select(&phy_vars_eNB[CC_id]->lte_frame_parms,phy_vars_eNB[CC_id]->proc[subframe].subframe_rx)==SF_S)){
      phy_procedures_eNB_S_RX(subframe,phy_vars_eNB[CC_id],abstraction_flag,r_type);
    }

    phy_vars_eNB[CC_id]->proc[subframe].frame_tx++;
    phy_vars_eNB[CC_id]->proc[subframe].frame_rx++;
    
    if (phy_vars_eNB[CC_id]->proc[subframe].frame_tx==MAX_FRAME_NUMBER) // defined in impl_defs_top.h
      phy_vars_eNB[CC_id]->proc[subframe].frame_tx=0;
    if (phy_vars_eNB[CC_id]->proc[subframe].frame_rx==MAX_FRAME_NUMBER)
      phy_vars_eNB[CC_id]->proc[subframe].frame_rx=0;
  }
  vcd_signal_dumper_dump_function_by_name(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_ENB_LTE,0);
  stop_meas(&phy_vars_eNB[0]->phy_proc);
}

