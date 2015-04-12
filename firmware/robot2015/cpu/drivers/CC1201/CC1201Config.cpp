#include "CC1201Radio.hpp"

CC1201Config::CC1201Config()
{
	CC1201Config::resetConfiguration(this);
}

CC1201Config::~CC1201Config() {}

void CC1201Config::resetConfiguration(CC1201Config* config)
{
	config->iocfg3 				= SMARTRF_SETTING_IOCFG3;	
	config->iocfg2 				= SMARTRF_SETTING_IOCFG2;
	config->iocfg1 				= SMARTRF_SETTING_IOCFG1;
	config->iocfg0 				= SMARTRF_SETTING_IOCFG0;
	config->sync3 				= SMARTRF_SETTING_SYNC3;
	config->sync2 				= SMARTRF_SETTING_SYNC2;
	config->sync1 				= SMARTRF_SETTING_SYNC1;
	config->sync0 				= SMARTRF_SETTING_SYNC0;
	config->syncCfg1 			= SMARTRF_SETTING_SYNC_CFG1;
	config->syncCfg0 			= SMARTRF_SETTING_SYNC_CFG2;
	config->deviationM 			= SMARTRF_SETTING_DEVIATION_M;
	config->modcfgDevE 			= SMARTRF_SETTING_MODCFG_DEV_E;
	config->dcfiltCfg 			= SMARTRF_SETTING_DCFILT_CFG;
	config->preambleCfg1 		= SMARTRF_SETTING_PREAMBLE_CFG1;
	config->preambleCfg0 		= SMARTRF_SETTING_PREAMBLE_CFG0;
	config->iqic; 				= SMARTRF_SETTING_IQIC;
	config->chanBw; 			= SMARTRF_SETTING_CHAN_BW;
	config->mdmcfg1; 			= SMARTRF_SETTING_MDMCFG1;
	config->mdmcfg0; 			= SMARTRF_SETTING_MDMCFG0;
	config->symbolRate2; 		// Symbol Rate Configuration Exponent and Mantissa [1..
	config->symbolRate1; 		// Symbol Rate Configuration Mantissa [15:8]
	config->symbolRate0; 		// Symbol Rate Configuration Mantissa [7:0]
	config->agcRef; 			// AGC Reference Level Configuration
	config->agcCsThr; 			// Carrier Sense Threshold Configuration
	config->agcGainAdjust; 		// RSSI Offset Configuration
	config->agcCfg3; 			// Automatic Gain Control Configuration Reg. 3
	config->agcCfg2; 			// Automatic Gain Control Configuration Reg. 2
	config->agcCfg1; 			// Automatic Gain Control Configuration Reg. 1
	config->agcCfg0; 			// Automatic Gain Control Configuration Reg. 0
	config->fifoCfg; 			// FIFO Configuration
	config->devAddr; 			// Device Address Configuration
	config->settlingCfg; 		// Frequency Synthesizer Calibration and Settling Con..
	config->fsCfg; 				// Frequency Synthesizer Configuration
	config->worCfg1; 			// eWOR Configuration Reg. 1
	config->worCfg0; 			// eWOR Configuration Reg. 0
	config->worEvent0Msb; 		// Event 0 Configuration MSB
	config->worEvent0Lsb; 		// Event 0 Configuration LSB
	config->rxdcmTime; 			// RX Duty Cycle Mode Configuration
	config->pktCfg2; 			// Packet Configuration Reg. 2
	config->pktCfg1; 			// Packet Configuration Reg. 1
	config->pktCfg0; 			// Packet Configuration Reg. 0
	config->rfendCfg1; 			// RFEND Configuration Reg. 1
	config->rfendCfg0; 			// RFEND Configuration Reg. 0
	config->paCfg1; 			// Power Amplifier Configuration Reg. 1
	config->paCfg0; 			// Power Amplifier Configuration Reg. 0
	config->askCfg; 			// ASK Configuration
	config->pktLen; 			// Packet Length Configuration
	config->ifMixCfg; 			// IF Mix Configuration
	config->freqoffCfg; 		// Frequency Offset Correction Configuration
	config->tocCfg; 			// Timing Offset Correction Configuration
	config->marcSpare; 			// MARC Spare
	config->ecgCfg; 			// External Clock Frequency Configuration
	config->mdmcfg2; 			// General Modem Parameter Configuration Reg. 2
	config->extCtrl; 			// External Control Configuration
	config->rccalFine; 			// RC Oscillator Calibration Fine
	config->rccalCoarse; 		// RC Oscillator Calibration Coarse
	config->rccalOffset; 		// RC Oscillator Calibration Clock Offset
	config->freqoff1; 			// Frequency Offset MSB
	config->freqoff0; 			// Frequency Offset LSB
	config->freq2; 				// Frequency Configuration [23:16]
	config->freq1; 				// Frequency Configuration [15:8]
	config->freq0; 				// Frequency Configuration [7:0]
	config->ifAdc2; 			// Analog to Digital Converter Configuration Reg. 2
	config->ifAdc1; 			// Analog to Digital Converter Configuration Reg. 1
	config->ifAdc0; 			// Analog to Digital Converter Configuration Reg. 0
	config->fsDig1; 			// Frequency Synthesizer Digital Reg. 1
	config->fsDig0; 			// Frequency Synthesizer Digital Reg. 0
	config->fsCal3; 			// Frequency Synthesizer Calibration Reg. 3
	config->fsCal2; 			// Frequency Synthesizer Calibration Reg. 2
	config->fsCal1; 			// Frequency Synthesizer Calibration Reg. 1
	config->fsCal0; 			// Frequency Synthesizer Calibration Reg. 0
	config->fsChp; 				// Frequency Synthesizer Charge Pump Configuration
	config->fsDivtwo; 			// Frequency Synthesizer Divide by 2
	config->fsDsm1; 			// FS Digital Synthesizer Module Configuration Reg. 1
	config->fsDsm0; 			// FS Digital Synthesizer Module Configuration Reg. 0
	config->fsDvc1; 			// Frequency Synthesizer Divider Chain Configuration ..
	config->fsDvc0; 			// Frequency Synthesizer Divider Chain Configuration ..
	config->fsLbi; 				// Frequency Synthesizer Local Bias Configuration
	config->fsPfd; 				// Frequency Synthesizer Phase Frequency Detector Con..
	config->fsPre; 				// Frequency Synthesizer Prescaler Configuration
	config->fsRegDivCml; 		// Frequency Synthesizer Divider Regulator Configurat..
	config->fsSpare; 			// Frequency Synthesizer Spare
	config->fsVco4; 			// FS Voltage Controlled Oscillator Configuration Reg..
	config->fsVco3; 			// FS Voltage Controlled Oscillator Configuration Reg..
	config->fsVco2; 			// FS Voltage Controlled Oscillator Configuration Reg..
	config->fsVco1; 			// FS Voltage Controlled Oscillator Configuration Reg..
	config->fsVco0; 			// FS Voltage Controlled Oscillator Configuration Reg..
	config->gbias6; 			// Global Bias Configuration Reg. 6
	config->gbias5; 			// Global Bias Configuration Reg. 5
	config->gbias4; 			// Global Bias Configuration Reg. 4
	config->gbias3; 			// Global Bias Configuration Reg. 3
	config->gbias2; 			// Global Bias Configuration Reg. 2
	config->gbias1; 			// Global Bias Configuration Reg. 1
	config->gbias0; 			// Global Bias Configuration Reg. 0
	config->ifamp; 				// Intermediate Frequency Amplifier Configuration
	config->lna; 				// Low Noise Amplifier Configuration
	config->rxmix; 				// RX Mixer Configuration
	config->xosc5; 				// Crystal Oscillator Configuration Reg. 5
	config->xosc4; 				// Crystal Oscillator Configuration Reg. 4
	config->xosc3; 				// Crystal Oscillator Configuration Reg. 3
	config->xosc2; 				// Crystal Oscillator Configuration Reg. 2
	config->xosc1; 				// Crystal Oscillator Configuration Reg. 1
	config->xosc0; 				// Crystal Oscillator Configuration Reg. 0
	config->analogSpare; 		// Analog Spare
	config->paCfg3; 			// Power Amplifier Configuration Reg. 3
	config->worTime1; 			// eWOR Timer Counter Value MSB
	config->worTime0; 			// eWOR Timer Counter Value LSB
	config->worCapture1; 		// eWOR Timer Capture Value MSB
	config->worCapture0; 		// eWOR Timer Capture Value LSB
	config->bist; 				// MARC Built-In Self-Test
	config->dcfiltoffsetI1; 	// DC Filter Offset I MSB
	config->dcfiltoffsetI0; 	// DC Filter Offset I LSB
	config->dcfiltoffsetQ1; 	// DC Filter Offset Q MSB
	config->dcfiltoffsetQ0; 	// DC Filter Offset Q LSB
	config->iqieI1; 			// IQ Imbalance Value I MSB
	config->iqieI0; 			// IQ Imbalance Value I LSB
	config->iqieQ1; 			// IQ Imbalance Value Q MSB
	config->iqieQ0; 			// IQ Imbalance Value Q LSB
	config->rssi1; 				// Received Signal Strength Indicator Reg. 1
	config->rssi0; 				// Received Signal Strength Indicator Reg.0
	config->marcstate; 			// MARC State
	config->lqiVal; 			// Link Quality Indicator Value
	config->pqtSyncErr; 		// Preamble and Sync Word Error
	config->demStatus; 			// Demodulator Status
	config->freqoffEst1; 		// Frequency Offset Estimate MSB
	config->freqoffEst0; 		// Frequency Offset Estimate LSB
	config->agcGain3; 			// Automatic Gain Control Reg. 3
	config->agcGain2; 			// Automatic Gain Control Reg. 2
	config->agcGain1; 			// Automatic Gain Control Reg. 1
	config->agcGain0; 			// Automatic Gain Control Reg. 0
	config->cfmRxDataOut; 		// Custom Frequency Modulation RX Data
	config->cfmTxDataIn; 		// Custom Frequency Modulation TX Data
	config->askSoftRxData; 		// ASK Soft Decision Output
	config->rndgen; 			// Random Number Generator Value
	config->magn2; 				// Signal Magnitude after CORDIC [16]
	config->magn1; 				// Signal Magnitude after CORDIC [15:8]
	config->magn0; 				// Signal Magnitude after CORDIC [7:0]
	config->ang1; 				// Signal Angular after CORDIC [9:8]
	config->ang0; 				// Signal Angular after CORDIC [7:0]
	config->chfiltI2; 			// Channel Filter Data Real Part [16]
	config->chfiltI1; 			// Channel Filter Data Real Part [15:8]
	config->chfiltI0; 			// Channel Filter Data Real Part [7:0]
	config->chfiltQ2; 			// Channel Filter Data Imaginary Part [16]
	config->chfiltQ1; 			// Channel Filter Data Imaginary Part [15:8]
	config->chfiltQ0; 			// Channel Filter Data Imaginary Part [7:0]
	config->gpioStatus; 		// General Purpose Input/Output Status
	config->fscalCtrl; 			// Frequency Synthesizer Calibration Control
	config->phaseAdjust; 		// Frequency Synthesizer Phase Adjust
	config->partnumber; 		// Part Number
	config->partversion; 		// Part Revision
	config->serialStatus; 		// Serial Status
	config->modemStatus1; 		// Modem Status Reg. 1
	config->modemStatus0;		// Modem Status Reg. 0
	config->marcStatus1; 		// MARC Status Reg. 1
	config->marcStatus0; 		// MARC Status Reg. 0
	config->paIfampTest; 		// Power Amplifier Intermediate Frequency Amplifier T..
	config->fsrfTest; 			// Frequency Synthesizer Test
	config->preTest; 			// Frequency Synthesizer Prescaler Test
	config->preOvr; 			// Frequency Synthesizer Prescaler Override
	config->adcTest; 			// Analog to Digital Converter Test
	config->dvcTest; 			// Digital Divider Chain Test
	config->atest; 				// Analog Test
	config->atestLvds; 			// Analog Test LVDS
	config->atestMode; 			// Analog Test Mode
	config->xoscTest1; 			// Crystal Oscillator Test Reg. 1
	config->xoscTest0; 			// Crystal Oscillator Test Reg. 0
	config->aes;				// AES
	config->mdmTest; 			// MODEM Test
	config->rxfirst; 			// RX FIFO Pointer First Entry
	config->txfirst; 			// TX FIFO Pointer First Entry
	config->rxlast; 			// RX FIFO Pointer Last Entry
	config->txlast; 			// TX FIFO Pointer Last Entry
	config->numTxbytes; 		// TX FIFO Status
	config->numRxbytes; 		// RX FIFO Status
	config->fifoNumTxbytes; 	// TX FIFO Status
	config->fifoNumRxbytes; 	// RX FIFO Status
	config->rxfifoPreBuf; 		// RX FIFO Status
	config->aesKey15; 			// Advanced Encryption Standard Key [127:120]
	config->aesKey14; 			// Advanced Encryption Standard Key [119:112]
	config->aesKey13; 			// Advanced Encryption Standard Key [111:104]
	config->aesKey12; 			// Advanced Encryption Standard Key [103:96]
	config->aesKey11; 			// Advanced Encryption Standard Key [95:88]
	config->aesKey10; 			// Advanced Encryption Standard Key [87:80]
	config->aesKey9; 			// Advanced Encryption Standard Key [79:72]
	config->aesKey8; 			// Advanced Encryption Standard Key [71:64]
	config->aesKey7; 			// Advanced Encryption Standard Key [63:56]
	config->aesKey6; 			// Advanced Encryption Standard Key [55:48]
	config->aesKey5; 			// Advanced Encryption Standard Key [47:40]
	config->aesKey4; 			// Advanced Encryption Standard Key [39:32]
	config->aesKey3; 			// Advanced Encryption Standard Key [31:24]
	config->aesKey2; 			// Advanced Encryption Standard Key [23:16]
	config->aesKey1; 			// Advanced Encryption Standard Key [15:8]
	config->aesKey0; 			// Advanced Encryption Standard Key [7:0]
	config->aesBuffer15; 		// Advanced Encryption Standard Buffer [127:120]
	config->aesBuffer14; 		// Advanced Encryption Standard Buffer [119:112]
	config->aesBuffer13; 		// Advanced Encryption Standard Buffer [111:104]
	config->aesBuffer12; 		// Advanced Encryption Standard Buffer [103:93]
	config->aesBuffer11; 		// Advanced Encryption Standard Buffer [95:88]
	config->aesBuffer10; 		// Advanced Encryption Standard Buffer [87:80]
	config->aesBuffer9; 		// Advanced Encryption Standard Buffer [79:72]
	config->aesBuffer8; 		// Advanced Encryption Standard Buffer [71:64]
	config->aesBuffer7; 		// Advanced Encryption Standard Buffer [63:56]
	config->aesBuffer6; 		// Advanced Encryption Standard Buffer [55:48]
	config->aesBuffer5; 		// Advanced Encryption Standard Buffer [47:40]
	config->aesBuffer4; 		// Advanced Encryption Standard Buffer [39:32]
	config->aesBuffer3; 		// Advanced Encryption Standard Buffer [31:24]
	config->aesBuffer2; 		// Advanced Encryption Standard Buffer [23:16]
	config->aesBuffer1; 		// Advanced Encryption Standard Buffer [15:8]
	config->aesBuffer0; 
}

CC1201* loadConfiguration(CC1201Config* config, CC1201* device)
{
	return NULL;
}

void loadLinkedDevice(void)
{

}
