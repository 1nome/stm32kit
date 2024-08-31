/**
 * @file	dma.h
 * @author	1nome (Adam Wiszczor)
 * 
 * @brief	An attempt at a universal dma library
 *			To setup a stream, first, set the addresses and number of transferred items using DMA_setup_addr()
 *			Then use DMA_setup_data() to config data sizes and pointer incrementation
 *			Finally, set the request channel and transfer direction using DMA_setup_behav()
 *			If necessary, enable circular or dual buffer mode while calling this function
 *			Additional config can be made with DMA_setup_interrupts(), _misc() and _fifo()
 *			After setting everything, enable the stream with DMA_enable()
 */
#include "boards.h"
#include <stdbool.h>

#define DMA_Stream DMA_Stream_TypeDef*

DMA_Stream DMA_Streams[] = {
	DMA1_Stream0,
	DMA1_Stream1,
	DMA1_Stream2,
	DMA1_Stream3,
	DMA1_Stream4,
	DMA1_Stream5,
	DMA1_Stream6,
	DMA1_Stream7,
	DMA2_Stream0,
	DMA2_Stream1,
	DMA2_Stream2,
	DMA2_Stream3,
	DMA2_Stream4,
	DMA2_Stream5,
	DMA2_Stream6,
	DMA2_Stream7
};

// dma is either 1 or 2, stream is from 0 to 7
DMA_Stream DMA_get_handle(uint8_t dma, uint8_t stream){
	// enable clock to dma if not already enabled
	// return 0 on wrong dma values
	switch(dma){
		case 1:
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
			break;
		case 2:
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
			break;
		default:
			return 0;
	}
	
	// return 0 on wrong stream values
	if(stream > 7){
		return 0;
	}
	return DMA_Streams[stream + (dma == 1 ? 0 : 8)];
}

void DMA_disable_handle(DMA_Stream s){
	s->CR &= ~DMA_SxCR_EN;
	while(s->CR & DMA_SxCR_EN);
}

// dma is either 1 or 2, stream and channel are from 0 to 7
// direct mode error, transfer error, half transfer, transfer complete interrupt enable
//  true/1 = enabled
void DMA_setup_interrupts(uint8_t dma, uint8_t stream, bool dmeie, bool teie, bool htie, bool tcie){
	DMA_Stream s = DMA_get_handle(dma, stream);
	if(s == 0){
		return;
	}
	DMA_disable_handle(s);

	s->CR &= ~(DMA_SxCR_DMEIE | DMA_SxCR_TEIE | DMA_SxCR_HTIE | DMA_SxCR_TCIE);
	s->CR |= (dmeie << DMA_SxCR_DMEIE_Pos)
			| (teie << DMA_SxCR_TEIE_Pos)
			| (htie << DMA_SxCR_HTIE_Pos)
			| (tcie << DMA_SxCR_TCIE_Pos);
}

// 8b, 16b, 32b
enum DMA_Size{
	Byte = 0b00,
	HalfWord,
	Word
};

// peripheral, memory ptr increment, size of those increments
//  true/1 = do increment
// pincos when true sets the peripheral offset to 4B; psize or msize don't have to be 4B
void DMA_setup_data(uint8_t dma, uint8_t stream, bool pinc, bool minc, enum DMA_Size psize, enum DMA_Size msize, bool pincos){
	DMA_Stream s = DMA_get_handle(dma, stream);
	if(s == 0){
		return;
	}
	DMA_disable_handle(s);
	
	s->CR &= ~(DMA_SxCR_PINC | DMA_SxCR_MINC | DMA_SxCR_PSIZE | DMA_SxCR_MSIZE | DMA_SxCR_PINCOS);
	s->CR |= (pinc << DMA_SxCR_PINC_Pos)
			| (minc << DMA_SxCR_MINC_Pos)
			| (psize << DMA_SxCR_PSIZE_Pos)
			| (msize << DMA_SxCR_MSIZE_Pos)
			| (pincos << DMA_SxCR_PINCOS_Pos);
}

enum DMA_Burst{
	Single = 0b00,
	Incr4,
	Incr8,
	Incr16
};

enum DMA_Prio{
	Low = 0b00,
	Medium,
	High,
	VeryHigh
};

// peripheral, memory burst transfer config; these only work if direct mode is disabled
// priority level; higher priorities get processed first (pHigh > pLow)
//  if two streams have the same priority, the lower-numbered is served first (p2 > p5)
// peripheral flow controler; when true/1, the peripheral sets the number of data items to be transferred
//  (do note that this works only with the SDIO peripheral)
void DMA_setup_misc(uint8_t dma, uint8_t stream, enum DMA_Burst pburst, enum DMA_Burst mburst, enum DMA_Prio pl, bool pfctrl){
	DMA_Stream s = DMA_get_handle(dma, stream);
	if(s == 0){
		return;
	}
	DMA_disable_handle(s);
	
	s->CR &= ~(DMA_SxCR_PBURST | DMA_SxCR_MBURST | DMA_SxCR_PL | DMA_SxCR_PFCTRL);
	s->CR |= (pburst << DMA_SxCR_PBURST_Pos)
			| (mburst << DMA_SxCR_MBURST_Pos)
			| (pl << DMA_SxCR_PL_Pos)
			| (pfctrl << DMA_SxCR_PFCTRL_Pos);
}

enum DMA_Dir{
	PerToMem = 0b00,
	MemToPer,
	MemToMem
};

// channel select; is from 0 to 7, sets the channel a stream will take transfer requests from
// direction; transfers to/from peripherals behave differently to those in memory
//  (when a peripheral is involved, it needs to request transfers; mem to mem runs full throttle)
//  (mem to mem also works only for dma2 and forbids circular, dual buffer and direct modes)
// circular mode; true/1 = enabled; when the ndtr(number of data) reaches 0,
//  ndtr is reset to its original value
// dual buffer mode; true/1 = enabled; automatically enables circular mode; when ndtr reaches 0,
//  transfers continue from/to the other memory address
// current target; sets the initial memory target (mem0 or mem1)
void DMA_setup_behav(uint8_t dma, uint8_t stream, uint8_t chsel, enum DMA_Dir dir, bool circ, bool dbm, bool ct){
	DMA_Stream s = DMA_get_handle(dma, stream);
	if(s == 0){
		return;
	}
	DMA_disable_handle(s);
	
	if(chsel > 7){
		chsel = 0;
	}
	if(dir == MemToMem && (dma == 1 || (circ | dbm))){
		return;
	}
	
	s->CR &= ~(DMA_SxCR_CHSEL | DMA_SxCR_DIR | DMA_SxCR_CIRC | DMA_SxCR_DBM | DMA_SxCR_CT);
	s->CR |= (chsel << DMA_SxCR_CHSEL_Pos)
			| (dir << DMA_SxCR_DIR_Pos)
			| (circ << DMA_SxCR_CIRC_Pos)
			| (dbm << DMA_SxCR_DBM_Pos)
			| (ct << DMA_SxCR_CT_Pos);
}

// periph - peripheral/source memory address
// mem0, mem1 - memory addresses, mem0 is always used, mem1 only in dbm
// ndtr - number of data tranfers before transfer completes / starts from beginning / changes memory
// also, setting an address/ndtr to null/0 will not change it
void DMA_setup_addr(uint8_t dma, uint8_t stream, void* periph, void* mem0, void* mem1, uint16_t ndtr){
	DMA_Stream s = DMA_get_handle(dma, stream);
	if(s == 0){
		return;
	}
	DMA_disable_handle(s);
	
	// confirm that stream is disabled
	s->CR &= ~DMA_SxCR_EN;
	while(s->CR & DMA_SxCR_EN);
	
	// set the peripheral address
	if(periph){
		s->PAR = (uint32_t)periph;
	}
	
	// set the memory addresses
	if(mem0){
		s->M0AR = (uint32_t)mem0;
	}
	if(mem1){
		s->M1AR = (uint32_t)mem1;
	}
	
	// config the number of item transfers
	if(ndtr){
		s->NDTR = ndtr;
	}
}

enum DMA_FifoTresh{
	Quarter = 0b00,
	Half,
	ThreeQ,
	Full_T
};

// fifo error interrupt enable; true/1 = enable
// direct mode disable; false/0 = direct mode enabled, it is enabled by default
//  best used for single transfers after a request
//  source and destination have to be the same size, burst transfers are not possible
// fifo threshhold selection - default is half; fifo size is 4 words (16B)
//  fifo has to have enough data for a mburst, so a 1/4 thresh and incr4 burst
//  isn't allowed for msize = half-word and word, but works for byte
void DMA_setup_fifo(uint8_t dma, uint8_t stream, bool feie, bool dmdis, enum DMA_FifoTresh fth){
	DMA_Stream s = DMA_get_handle(dma, stream);
	if(s == 0){
		return;
	}
	DMA_disable_handle(s);
	
	s->FCR &= ~(DMA_SxFCR_FEIE | DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
	s->CR |= (feie << DMA_SxFCR_FEIE_Pos)
			| (dmdis << DMA_SxFCR_DMDIS_Pos)
			| (fth << DMA_SxFCR_FTH_Pos);
}

void DMA_enable(uint8_t dma, uint8_t stream){
	DMA_Stream s = DMA_get_handle(dma, stream);
	if(s == 0){
		return;
	}
	
	s->CR |= DMA_SxCR_EN;
}

void DMA_disable(uint8_t dma, uint8_t stream){
	DMA_Stream s = DMA_get_handle(dma, stream);
	if(s == 0){
		return;
	}
	
	DMA_disable_handle(s);
}

// clear fifo error, direct mode error, transfer error, half transfer, transfer complete interrupt flag
//  true/1 = clear flag
void DMA_clear_int_flags(uint8_t dma, uint8_t stream, bool cfeif, bool cdmeif, bool cteif, bool chtif, bool ctcif){
	uint32_t write = cfeif
				| (cdmeif << DMA_LIFCR_CDMEIF0_Pos)
				| (cteif << DMA_LIFCR_CTEIF0_Pos)
				| (chtif << DMA_LIFCR_CHTIF0_Pos)
				| (ctcif << DMA_LIFCR_CTCIF0_Pos);
	
	switch(dma){
		case 1:
			if(stream < 4){
				DMA1->LIFCR |= write << (6 * stream);
			}
			stream -= 4;
			if(stream < 4){
				DMA1->HIFCR |= write << (6 * stream);
			}
			return;
		case 2:
			if(stream < 4){
				DMA2->LIFCR |= write << (6 * stream);
			}
			stream -= 4;
			if(stream < 4){
				DMA2->HIFCR |= write << (6 * stream);
			}
			return;
		default:
			return;
	}
}

// from lsb to msb: fifo error int flag, junk, direct mode error, transfer error,
//  half transfer, transfer complete interrupt flag, junk, junk
//  to get the value of a single flag, & the result with a macro
//  tcif = *result* & DMA_LISR_TCIF0 (always use lisr_*int*0 or hisr_*int*4)
uint8_t DMA_get_int_flags(uint8_t dma, uint8_t stream){
	switch(dma){
		case 1:
			if(stream < 4){
				return DMA1->LISR >> (6 * stream);
			}
			stream -= 4;
			if(stream < 4){
				return DMA1->HISR >> (6 * stream);
			}
			return 0;
		case 2:
			if(stream < 4){
				return DMA2->LISR >> (6 * stream);
			}
			stream -= 4;
			if(stream < 4){
				return DMA2->HISR >> (6 * stream);
			}
			return 0;
		default:
			return 0;
	}
}

// if dbm is enabled on a running stream, returns the memory pointer used at the moment
bool DMA_get_ct(uint8_t dma, uint8_t stream){
	DMA_Stream s = DMA_get_handle(dma, stream);
	if(s == 0){
		return false;
	}
	
	return s->CR & DMA_SxCR_CT;
}

// useful if you want to pause a transfer and then continue from the last location
//  add the difference of the original value and this to the pointer(s)
uint16_t DMA_get_remaining_ndt(uint8_t dma, uint8_t stream){
	DMA_Stream s = DMA_get_handle(dma, stream);
	if(s == 0){
		return false;
	}
	
	return s->NDTR;
}

enum DMA_FifoStatus{
	LessThanQuarter = 0b000,
	LessThanHalf,
	LessThanThreeQ,
	LessThanFull,
	Empty,
	Full_S
};

uint8_t DMA_get_fifo_status(uint8_t dma, uint8_t stream){
	DMA_Stream s = DMA_get_handle(dma, stream);
	if(s == 0){
		return false;
	}
	
	return (s->FCR & DMA_SxFCR_FS) >> DMA_SxFCR_FS_Pos;
}
