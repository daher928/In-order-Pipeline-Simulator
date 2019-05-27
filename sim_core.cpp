
/* This file holds the implementation of the CPU pipeline core simulator */

#include "sim_api.h"
#include <exception>
#include <cstdio>


#ifdef __cplusplus
extern "C" {
#endif

	static SIM_coreState simulator;

	static int32_t destination[SIM_PIPELINE_DEPTH] = { -1 };	//reg dest index
	static int32_t ex_result[SIM_PIPELINE_DEPTH] = { -1 };
	static int zero_res[SIM_PIPELINE_DEPTH] = { -1 };
	static int load_val[SIM_PIPELINE_DEPTH] = { -1 };
	static int32_t prev_wb_add;
	static int32_t prev_wb_val;
	static int read_failure;
	static int pause;
	static int pause_for_hazard;
	static int hazard_detect;
	static int hazard_stage;
	static int br_taken;
	static int32_t br_taken_address;
	static int load_fW_stall;
	static int LFW;

	static void updatePipeStage(int stage) {//given a stage index, this function fetches the next instruction into this stage (excluding IF stage)
											//if stage is IF/ID/EX/MEM, check pausing due to read failure

		if (forwarding==0 && ( pause != 1 && ((pause_for_hazard == 0) || (pause_for_hazard == 1 && stage >= 3)) ) ) {
			(simulator.pipeStageState[stage]).cmd.opcode = (simulator.pipeStageState[stage - 1]).cmd.opcode;
			(simulator.pipeStageState[stage]).cmd.dst = (simulator.pipeStageState[stage - 1]).cmd.dst;
			(simulator.pipeStageState[stage]).cmd.src1 = (simulator.pipeStageState[stage - 1]).cmd.src1;
			(simulator.pipeStageState[stage]).cmd.src2 = (simulator.pipeStageState[stage - 1]).cmd.src2;
			(simulator.pipeStageState[stage]).cmd.isSrc2Imm = (simulator.pipeStageState[stage - 1]).cmd.isSrc2Imm;
			(simulator.pipeStageState[stage]).src1Val = (simulator.pipeStageState[stage - 1]).src1Val;
			(simulator.pipeStageState[stage]).src2Val = (simulator.pipeStageState[stage - 1]).src2Val;
			destination[stage] = destination[stage - 1];
			ex_result[stage] = ex_result[stage - 1];
			zero_res[stage] = zero_res[stage - 1];
			load_val[stage] = load_val[stage - 1];
		}

		if (forwarding==0 && (pause == 1 || pause_for_hazard == 1) ){
			if (pause == 1) {
			(simulator.pipeStageState[4]).cmd.opcode = CMD_NOP;
			(simulator.pipeStageState[4]).cmd.dst = 0;
			(simulator.pipeStageState[4]).cmd.src1 = 0;
			(simulator.pipeStageState[4]).cmd.src2 = 0;
			(simulator.pipeStageState[4]).cmd.isSrc2Imm = 0;
			(simulator.pipeStageState[4]).src1Val = 0;
			(simulator.pipeStageState[4]).src2Val = 0;
			destination[4] = -1;
			load_val[4] = -1;
			ex_result[4] = -1;
			zero_res[4] = -1;
			}
			if (pause_for_hazard == 1) {
				if (stage == 2) {
				(simulator.pipeStageState[2]).cmd.opcode = CMD_NOP;
				(simulator.pipeStageState[2]).cmd.dst = 0;
				(simulator.pipeStageState[2]).cmd.src1 = 0;
				(simulator.pipeStageState[2]).cmd.src2 = 0;
				(simulator.pipeStageState[2]).cmd.isSrc2Imm = 0;
				(simulator.pipeStageState[2]).src1Val = 0;
				(simulator.pipeStageState[2]).src2Val = 0;
				destination[2] = -1;
				load_val[2] = -1;
				ex_result[2] = -1;
				zero_res[2] = -1;
			}
			if (stage<=1)
				hazard_stage--;
			}
		}

		if (forwarding==1 && ( pause != 1 && ( (load_fW_stall == 0) || (load_fW_stall == 1 && stage >=3)) ) ) {
			(simulator.pipeStageState[stage]).cmd.opcode = (simulator.pipeStageState[stage - 1]).cmd.opcode;
			(simulator.pipeStageState[stage]).cmd.dst = (simulator.pipeStageState[stage - 1]).cmd.dst;
			(simulator.pipeStageState[stage]).cmd.src1 = (simulator.pipeStageState[stage - 1]).cmd.src1;
			(simulator.pipeStageState[stage]).cmd.src2 = (simulator.pipeStageState[stage - 1]).cmd.src2;
			(simulator.pipeStageState[stage]).cmd.isSrc2Imm = (simulator.pipeStageState[stage - 1]).cmd.isSrc2Imm;
			(simulator.pipeStageState[stage]).src1Val = (simulator.pipeStageState[stage - 1]).src1Val;
			(simulator.pipeStageState[stage]).src2Val = (simulator.pipeStageState[stage - 1]).src2Val;
			destination[stage] = destination[stage - 1];
			ex_result[stage] = ex_result[stage - 1];
			zero_res[stage] = zero_res[stage - 1];
			load_val[stage] = load_val[stage - 1];
		}
		if (forwarding==1 && (pause == 1 || load_fW_stall == 1) ){
			if (pause == 1) {
				(simulator.pipeStageState[4]).cmd.opcode = CMD_NOP;
				(simulator.pipeStageState[4]).cmd.dst = 0;
				(simulator.pipeStageState[4]).cmd.src1 = 0;
				(simulator.pipeStageState[4]).cmd.src2 = 0;
				(simulator.pipeStageState[4]).cmd.isSrc2Imm = 0;
				(simulator.pipeStageState[4]).src1Val = 0;
				(simulator.pipeStageState[4]).src2Val = 0;
				destination[4] = -1;
				load_val[4] = -1;
				ex_result[4] = -1;
				zero_res[4] = -1;
			}
			else if (load_fW_stall == 1) {
				if (stage == 2) {
					(simulator.pipeStageState[2]).cmd.opcode = CMD_NOP;
					(simulator.pipeStageState[2]).cmd.dst = 0;
					(simulator.pipeStageState[2]).cmd.src1 = 0;
					(simulator.pipeStageState[2]).cmd.src2 = 0;
					(simulator.pipeStageState[2]).cmd.isSrc2Imm = 0;
					(simulator.pipeStageState[2]).src1Val = 0;
					(simulator.pipeStageState[2]).src2Val = 0;
					destination[2] = -1;
					load_val[2] = -1;
					ex_result[2] = -1;
					zero_res[2] = -1;
				}
			}
		}

		return;
	}

	static void Br_taken(int stage) {
		//if the branch is taken, update the Pc for the new address, and flush the IF, ID, EXE
	//	printf("BRANCH NOW !");
		//simulator.pc = newPc;
		(simulator.pipeStageState[stage]).cmd.opcode = CMD_NOP;
		(simulator.pipeStageState[stage]).cmd.dst = 0;
		(simulator.pipeStageState[stage]).cmd.src1 = 0;
		(simulator.pipeStageState[stage]).cmd.src2 = 0;
		(simulator.pipeStageState[stage]).cmd.isSrc2Imm = 0;
		(simulator.pipeStageState[stage]).src1Val = 0;
		(simulator.pipeStageState[stage]).src2Val = 0;
		destination[stage] = -1;
		load_val[stage] = -1;
		ex_result[stage] = -1;
		zero_res[stage] = -1;

	}

	void Data_flush(int stage) {
		if (split_regfile == 1 && forwarding == 0)
			stage++;
		pause_for_hazard = 1;
		hazard_stage = 5 - stage;
		return;
	}

	void IF() {
		if (br_taken == 1) {
			simulator.pc = br_taken_address;
			SIM_MemInstRead(simulator.pc, &(simulator.pipeStageState[0].cmd));
			pause_for_hazard = 0;
			//hazard_detect = 0;
			return;
		}
		if ( pause == 1 || hazard_detect == 1) {
			return;
		}
		if (load_fW_stall == 1) {
			return;
		}

		simulator.pc += 4;
		SIM_MemInstRead(simulator.pc, &(simulator.pipeStageState[0].cmd));
	}

	void ID() {
		updatePipeStage(1);//import next instruction into ID stage
		if (br_taken == 1) {
			Br_taken(1);
			return;
		}

		SIM_cmd_opcode command = simulator.pipeStageState[1].cmd.opcode;
		int src1 = simulator.pipeStageState[1].cmd.src1;
		int32_t src2 = simulator.pipeStageState[1].cmd.src2;
		int dst = simulator.pipeStageState[1].cmd.dst;
		bool isSrc2Imm = simulator.pipeStageState[1].cmd.isSrc2Imm;
		if (command == CMD_ADD || command == CMD_SUB) {//if command is ADD or SUB or BEQ or BNEQ, values found in registers 
			simulator.pipeStageState[1].src1Val = simulator.regFile[src1];
			if (isSrc2Imm == false)//if src2 is a register
				simulator.pipeStageState[1].src2Val = simulator.regFile[src2];
			else //if src2 is immediate
				simulator.pipeStageState[1].src2Val = src2;
			destination[1] = simulator.regFile[dst];
		}
		if (command == CMD_BREQ || command == CMD_BRNEQ || command == CMD_BR) { 
			simulator.pipeStageState[1].src1Val = simulator.regFile[src1];
			simulator.pipeStageState[1].src2Val = simulator.regFile[src2];
			destination[1] = simulator.regFile[dst];
		//	printf("  destination of BR in ID=%d  ", destination[1]);
		}
		if (command == CMD_STORE || command == CMD_LOAD) { //if command is LOAD or STORE, src2 might be immidiate
			simulator.pipeStageState[1].src1Val = simulator.regFile[src1];
			if (isSrc2Imm == false)//if src2 is a register
				simulator.pipeStageState[1].src2Val = simulator.regFile[src2];
			else //if src2 is immediate
				simulator.pipeStageState[1].src2Val = src2;
			destination[1] = simulator.regFile[dst];
		}
		if (command == CMD_NOP) {
			(simulator.pipeStageState[0]).src1Val = 0;
			(simulator.pipeStageState[0]).src2Val = 0;
			destination[1] = -1;
		}
		if (read_failure == 1 || pause_for_hazard == 1) {
			return;
		}

		int stop_stage= 5;
		if (split_regfile == 1 && forwarding==0)
			stop_stage = 4;

		if (forwarding == 0){//if no forwarding, check data hazard

			for (int i = 2; i < stop_stage; i++) {//DATA HAZARD DETECTION
				SIM_cmd_opcode i_op = simulator.pipeStageState[i].cmd.opcode;
				int i_dest = simulator.pipeStageState[i].cmd.dst;
				if ((i_op == CMD_ADD || i_op == CMD_SUB || i_op == CMD_LOAD) &&
					(  src1 == i_dest || src2 == i_dest || (command == CMD_STORE && dst==i_dest) ) )  {
					Data_flush(i);
					break;
				}
				if ((i_op == CMD_ADD || i_op == CMD_SUB || i_op == CMD_LOAD) &&
					(command == CMD_BR || command == CMD_BREQ || command == CMD_BRNEQ)) {
					if (dst == i_dest) {
						Data_flush(i);
						break;
					}
				}
			}
		}

		/* CHECH FOR MEM -> EX forwarding */
		SIM_cmd_opcode mem_op = simulator.pipeStageState[2].cmd.opcode;
		int mem_dest = simulator.pipeStageState[2].cmd.dst;

		if (forwarding == 1) {
			if (mem_op == CMD_LOAD && (src1 == mem_dest || (src2 == mem_dest) || ((command==CMD_BR || command==CMD_BREQ || command==CMD_BRNEQ) && dst==mem_dest))) {
				LFW = 1;
				return;
			}
		}
		if(command != CMD_BREQ && command != CMD_BRNEQ && command != CMD_BR && command != CMD_STORE && command != CMD_LOAD)
			destination[1] = -1;
	}

	void EX() {
		if (br_taken == 1) {
			Br_taken(2);
			return;
		}
		updatePipeStage(2);

		SIM_cmd_opcode command = simulator.pipeStageState[2].cmd.opcode;
		int32_t val1 = simulator.pipeStageState[2].src1Val;
		int32_t val2 = simulator.pipeStageState[2].src2Val;
		int src1 = simulator.pipeStageState[2].cmd.src1;
		int32_t src2 = simulator.pipeStageState[2].cmd.src2;
		int dst = simulator.pipeStageState[2].cmd.dst;

		// *************************** NO FORWARDING HERE **************************
		if (forwarding == 0) {
			switch (command) {
			case CMD_ADD:
				ex_result[2] = val1 + val2;
				break;
			case CMD_SUB:
				ex_result[2] = val1 - val2;
				break;
			case CMD_LOAD:
				ex_result[2] = val1 + val2;
				break;
			case CMD_STORE:
				ex_result[2] = destination[2] + val2;	// ex_result[2] = dst + val2
				break;
			case CMD_BR:
				ex_result[2] = destination[2] + simulator.pc;	// calculate the address for branch
				//printf("   address for BR here is %d   ,   destination[2]=%d    , pc=%d", ex_result[2], destination[2], simulator.pc);
				break;
			case CMD_BREQ:
				zero_res[2] = (val1 - val2 == 0) ? 1 : 0;	// zero if src1==src2
				ex_result[2] = destination[2] + simulator.pc; //branch address
				break;
			case CMD_BRNEQ:
				zero_res[2] = (val1 - val2 != 0) ? 0 : 1;	// zero if src1==src2
				ex_result[2] = destination[2] + simulator.pc; //branch address
				break;
			}
		}


		// **************************** FORWARDING HERE ********************************
		else if (forwarding == 1) {

				/* CHECH FOR WB -> EX forwarding */
				SIM_cmd_opcode wb_op = simulator.pipeStageState[4].cmd.opcode;
				int wb_dest = simulator.pipeStageState[4].cmd.dst;
				if (wb_op == CMD_LOAD) {
					if (src1 == wb_dest) { //fw
						simulator.pipeStageState[2].src1Val = load_val[4];
					}
					if (src2 == wb_dest) { //fw
						simulator.pipeStageState[2].src2Val = load_val[4];
					}
					if ((command == CMD_BR || command == CMD_BREQ || command == CMD_BRNEQ) && (dst == wb_dest)) {
						destination[2] = load_val[4];
					}/////////////////
				}
				if (wb_op == CMD_ADD || wb_op == CMD_SUB) {
					if (src1 == wb_dest) { //fw
						simulator.pipeStageState[2].src1Val = ex_result[4];
					}
					if (src2 == wb_dest) { //fw
						simulator.pipeStageState[2].src2Val = ex_result[4];
					}
					if (command == CMD_STORE && dst == wb_dest ) {
						destination[2] = ex_result[4];
					}
					if ((command == CMD_BR || command == CMD_BREQ || command == CMD_BRNEQ) && (dst == wb_dest)) {
						destination[2] = ex_result[4];
					}
				}

				/* CHECH FOR MEM -> EX forwarding */
				SIM_cmd_opcode mem_op = simulator.pipeStageState[3].cmd.opcode;
				int mem_dest = simulator.pipeStageState[3].cmd.dst;
				int degel = 0;
				if (mem_op == CMD_ADD || mem_op == CMD_SUB) {
					if (src1 == mem_dest) { //fw
						degel = 1;
						simulator.pipeStageState[2].src1Val = ex_result[3];
					}
					if (src2 == mem_dest) { //fw
						degel = 1;
						simulator.pipeStageState[2].src2Val = ex_result[3];
					}
					if (command == CMD_STORE && dst == mem_dest) {
						destination[2] = ex_result[3];
						degel = 1;
					}
					if ((command == CMD_BR || command == CMD_BREQ || command == CMD_BRNEQ) && (dst == mem_dest)) {
						degel = 1;
						destination[2] = ex_result[3];
					}
				}
				if (mem_op == CMD_LOAD && load_fW_stall == 1) {
					degel = 1;
					destination[2] = load_val[4];
				}

		}

		val1 = simulator.pipeStageState[2].src1Val;
		val2 = simulator.pipeStageState[2].src2Val;
		switch (command) {
		case CMD_ADD:
			ex_result[2] = val1 + val2;
			break;
		case CMD_SUB:
			ex_result[2] = val1 - val2;
			break;
		case CMD_LOAD:
			ex_result[2] = val1 + val2;
			break;
		case CMD_STORE:
			ex_result[2] = destination[2] + val2;	// ex_result[2] = dst + val2
			break;
		case CMD_BR:
			ex_result[2] = destination[2] + simulator.pc;	// calculate the address for branch
			break;
		case CMD_BREQ:
			zero_res[2] = (val1 - val2 == 0) ? 1 : 0;	// zero if src1==src2
			ex_result[2] = destination[2] + simulator.pc; //branch address
			break;
		case CMD_BRNEQ:
			zero_res[2] = (val1 - val2 != 0) ? 0 : 1;	// zero if src1==src2
			ex_result[2] = destination[2] + simulator.pc; //branch address
			break;
		}

	}

	void MEM() {
		if (br_taken == 1) {
			Br_taken(3);
			return;
		}
		updatePipeStage(3);

		SIM_cmd_opcode command = simulator.pipeStageState[3].cmd.opcode;
		int32_t val1 = simulator.pipeStageState[3].src1Val;
		int32_t val2 = simulator.pipeStageState[3].src2Val;
		if (command == CMD_BR || (command == CMD_BREQ && zero_res[3] == 1) || (command == CMD_BRNEQ && zero_res[3] == 0)) {
			br_taken = 2;
			br_taken_address = ex_result[3];
			return;
		}

		if (command == CMD_STORE) {
			SIM_MemDataWrite(ex_result[3], val1);
			return;
		}

		if (command == CMD_LOAD) {
			if (SIM_MemDataRead(ex_result[3], &(load_val[3])) == -1) {	//the value loaded saved in load_val[3]
				read_failure = 1;
			}
			return;
		}
	}

	void WB() {
		updatePipeStage(4);
		SIM_cmd_opcode command = simulator.pipeStageState[4].cmd.opcode;
		int32_t val1 = simulator.pipeStageState[4].src1Val;
		int32_t val2 = simulator.pipeStageState[4].src2Val;
		int dst = simulator.pipeStageState[4].cmd.dst;
		if (split_regfile == 0 && forwarding == 0) {			//NO SPLIT REGFILE
			if (command == CMD_ADD || command == CMD_SUB) {
				prev_wb_add = ex_result[4];
				return;
			}
			if (command == CMD_LOAD) {
				prev_wb_val = load_val[4];
				return;
			}
		}												
		if (command == CMD_ADD || command == CMD_SUB) {		//SPLIT REGFILE
			simulator.regFile[dst] = ex_result[4];
			return;
		}
		if (command == CMD_LOAD) {
			simulator.regFile[dst] = load_val[4];
			return;
		}

	}

	void do_prev_wb() {
		SIM_cmd_opcode command = simulator.pipeStageState[4].cmd.opcode;
		int32_t val1 = simulator.pipeStageState[4].src1Val;
		int32_t val2 = simulator.pipeStageState[4].src2Val;
		int dst = simulator.pipeStageState[4].cmd.dst;

		if (command == CMD_ADD || command == CMD_SUB) {
			simulator.regFile[dst] = prev_wb_add;
			return;
		}
		if (command == CMD_LOAD) {
			simulator.regFile[dst] = prev_wb_val;
			return;
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
		simulator.pc = 0;
		for (int i = 0; i < SIM_REGFILE_SIZE; i++) {
			simulator.regFile[i] = 0;
		}
		// IF initializing with the instruction in address 0x00
		SIM_MemInstRead(0X0, &(simulator.pipeStageState[0].cmd));
		(simulator.pipeStageState[0]).src1Val = 0;
		(simulator.pipeStageState[0]).src2Val = 0;

		for (int i = 1; i < SIM_PIPELINE_DEPTH; i++) {
			(simulator.pipeStageState[i]).cmd.opcode = CMD_NOP;
			(simulator.pipeStageState[i]).cmd.dst = 0;
			(simulator.pipeStageState[i]).cmd.src1 = 0;
			(simulator.pipeStageState[i]).cmd.src2 = 0;
			(simulator.pipeStageState[i]).cmd.isSrc2Imm = 0;
		}
		return 0;
	}

	/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
	This function is expected to update the core pipeline given a clock cycle event.
	*/
	void SIM_CoreClkTick() {

		LFW = 0;
		if (forwarding == 1)
			split_regfile = 1;

		read_failure = 0;
		if(br_taken != 0)
			br_taken--; 
		
		if(split_regfile == 0) {
			do_prev_wb();
		}
		WB();
		MEM();
		EX();
		ID();
		IF();
		pause = 0;
		hazard_detect = 0;
		load_fW_stall = 0;
		if (LFW==1)
			load_fW_stall = 1;
		if (read_failure == 1) {
			pause = 1;
		}
		if (hazard_stage == 0) {
			pause_for_hazard = 0;
		}
		if (pause_for_hazard == 1) {
			hazard_detect = 1;
		}
	
	}

	/*! SIM_CoreGetState: Return the current core (pipeline) internal state
	curState: The returned current pipeline state
	The function will return the state of the pipe at the end of a cycle
	*/
	void SIM_CoreGetState(SIM_coreState *curState) {
		*curState = simulator;
	}

#ifdef __cplusplus
}
#endif
