/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"

SIM_coreState state;
int32_t stage_dest_val[SIM_PIPELINE_DEPTH];
bool last_read_mem_failed;

int32_t store_cmd_temp_val;
uint32_t store_cmd_temp_addr;
bool last_cycle_store_cmd = false;;

int32_t regfile_temp_val;
uint32_t regfile_temp_idx;
bool last_cycle_change_regfile = false;

#define STAGE_IF    0
#define STAGE_ID    1
#define STAGE_EX    2
#define STAGE_MEM   3
#define STAGE_WB    4


void reset_stage(uint32_t stage) {
  memset(&state.pipeStageState[stage], 0, sizeof(PipeStageState));
  stage_dest_val[stage] = 0;
}

void fetch_instruction() {
  SIM_MemInstRead(state.pc, &state.pipeStageState[STAGE_IF].cmd);
  stage_dest_val[STAGE_IF] = 0;
}

void write_regfile() {
  if (last_cycle_change_regfile) {
    state.regFile[regfile_temp_idx] = regfile_temp_val;
     last_cycle_change_regfile = false;
  }
}
void write_last_cycle_vals() {
  write_regfile();
  if (last_cycle_store_cmd) {
    SIM_MemDataWrite(store_cmd_temp_addr, store_cmd_temp_val);
    last_cycle_store_cmd = false;
  }
}

/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  - The value of IF is the instuction in address 0x0
  \returns 0 on success. <0 in case of initialization failure.
*/
int SIM_CoreReset(void) {
  last_read_mem_failed = false;
  memset(&state, 0, sizeof(SIM_coreState));
  memset(&stage_dest_val, 0, SIM_PIPELINE_DEPTH * sizeof(int32_t));
  fetch_instruction();
  return 0;
}

void DoIDStage() {
  PipeStageState* current_stage = &state.pipeStageState[STAGE_ID];

  current_stage->src1Val = state.regFile[current_stage->cmd.src1];
  if (current_stage->cmd.isSrc2Imm) {
    current_stage->src2Val = current_stage->cmd.src2;
  }
  else {
    current_stage->src2Val = state.regFile[current_stage->cmd.src2];
  }
  
  if (current_stage->cmd.opcode == CMD_STORE ||
      current_stage->cmd.opcode == CMD_BR ||
      current_stage->cmd.opcode == CMD_BREQ ||
      current_stage->cmd.opcode == CMD_BRNEQ) {
    stage_dest_val[STAGE_ID] = state.regFile[current_stage->cmd.dst];
  }
}

void DoEXStage() {
  PipeStageState* current_stage = &state.pipeStageState[STAGE_EX];
  switch(current_stage->cmd.opcode) {
    case CMD_ADD:
      stage_dest_val[STAGE_EX] = current_stage->src1Val + current_stage->src2Val;
      break;
    case CMD_ADDI:
      stage_dest_val[STAGE_EX] = current_stage->src1Val + current_stage->src2Val;
      break;
    case CMD_SUB:
      stage_dest_val[STAGE_EX] = current_stage->src1Val - current_stage->src2Val;
      break;
    case CMD_SUBI:
      stage_dest_val[STAGE_EX] = current_stage->src1Val - current_stage->src2Val;
      break;
    default:
      break;
  }
}
void DoMemStage() {
// TODO: ADD BREQ, BNEQ, BR
// TODO: Implement Forwarding to EX
  PipeStageState* current_stage = &state.pipeStageState[STAGE_MEM];
  switch(current_stage->cmd.opcode) {
    case CMD_LOAD:
      if (0 != SIM_MemDataRead(current_stage->src1Val + current_stage->src2Val, &stage_dest_val[STAGE_MEM])) {
        last_read_mem_failed = true;
      }
      break;
    case CMD_STORE:
      store_cmd_temp_addr = stage_dest_val[STAGE_MEM] + current_stage->src2Val;
      store_cmd_temp_val = current_stage->src1Val;
      last_cycle_store_cmd = true;
      break;
    default:
      break;
  }
}

void DoWBStage() {
  PipeStageState* current_stage = &state.pipeStageState[STAGE_WB];

  if (state.pipeStageState[STAGE_WB].cmd.dst == 0) return;
  
  if (CMD_LOAD == current_stage->cmd.opcode || 
      (CMD_NOP < current_stage->cmd.opcode && CMD_SUBI >= current_stage->cmd.opcode)) {
    regfile_temp_idx = current_stage->cmd.dst;
    regfile_temp_val = stage_dest_val[STAGE_WB];
    last_cycle_change_regfile = true;
  }
  
  if (CMD_LOAD == current_stage->cmd.opcode) {
    if (forwarding) {
      PipeStageState* ex_stage = &state.pipeStageState[STAGE_EX];

      if (ex_stage->cmd.src1 == current_stage->cmd.dst) {
        ex_stage->src1Val = state.regFile[current_stage->cmd.dst];
      }
      if (ex_stage->cmd.src2 == current_stage->cmd.dst &&
          !ex_stage->cmd.isSrc2Imm) {
        ex_stage->src2Val = state.regFile[current_stage->cmd.dst];
      }
    }
  }
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {

  write_last_cycle_vals();

  if (last_read_mem_failed) {
    reset_stage(STAGE_WB);
    last_read_mem_failed = false;
  }

  else if (forwarding) {
    //TODO: Implement pipe forwarding
  }

  else { //MEM Didn't fail & Stalling or Split Regfile 
    bool data_hazard_detected = false;
    int last_stage_cannot_forward = STAGE_WB;
    if (split_regfile) {
      last_stage_cannot_forward = STAGE_MEM;
    }
    for (int i = STAGE_EX; i <= last_stage_cannot_forward; i ++){ 
      if (state.pipeStageState[i].cmd.opcode > CMD_NOP && state.pipeStageState[i].cmd.opcode <= CMD_LOAD) { //command that changes regfile
        int stage_dest = state.pipeStageState[i].cmd.dst;
        if (state.pipeStageState[STAGE_ID].cmd.src1 == stage_dest ||

            (!state.pipeStageState[STAGE_ID].cmd.isSrc2Imm 
              && state.pipeStageState[STAGE_ID].cmd.src2 == stage_dest) ||

              (state.pipeStageState[STAGE_ID].cmd.dst == stage_dest && //Some commands need to read dst register at Decode stage
                (state.pipeStageState[STAGE_ID].cmd.opcode == CMD_STORE ||
                 state.pipeStageState[STAGE_ID].cmd.opcode == CMD_BREQ ||
                 state.pipeStageState[STAGE_ID].cmd.opcode == CMD_BR ||
                 state.pipeStageState[STAGE_ID].cmd.opcode == CMD_BRNEQ)
              )) {
              data_hazard_detected = true;
              break;         
        }
      }
    }
    for (int i = STAGE_WB; i >= STAGE_EX; i--) { 
      state.pipeStageState[i] = state.pipeStageState[i-1];
      stage_dest_val[i] = stage_dest_val[i-1];
    }
    if (data_hazard_detected) {
      reset_stage(STAGE_EX);
    }
    else {
      state.pipeStageState[STAGE_ID] = state.pipeStageState[STAGE_IF];
      stage_dest_val[STAGE_ID] = stage_dest_val[STAGE_IF];
      state.pc += 4;
    }
  }

  
  DoWBStage();
  if (split_regfile) {
   write_regfile();
  }
  DoMemStage();
  DoEXStage();
  DoIDStage();
  fetch_instruction();
}

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {
  memcpy(curState, &state, sizeof(SIM_coreState));
}

