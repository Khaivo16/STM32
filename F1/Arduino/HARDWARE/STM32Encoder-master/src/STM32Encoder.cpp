#include <STM32Encoder.h>


//void empty handler for exti
void empty_handler(void) {
	//do nothing
}

//exti handlers
static void (*_isrptr_extia) (void)=empty_handler;					//tim isr handler ptr
static void (*_isrptr_extib) (void)=empty_handler;					//tim isr handler ptr
static void (*_isrptr_extic) (void)=empty_handler;					//tim isr handler ptr
static void (*_isrptr_extid) (void)=empty_handler;					//tim isr handler ptr
static void (*_isrptr_extie) (void)=empty_handler;					//tim isr handler ptr



STM32Encoder::STM32Encoder (TIM_TypeDef * Timer, unsigned char count_mode, unsigned int prescaler, unsigned int pulsePerRevolution) {
	_TIMER = Timer; //Timer to be used. 
	//timer_init(_TIMER);//use a general use timer.

	// Enable TIM clock
#if defined(TIM1_BASE)
  if (Timer == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();		
  }
#endif
#if defined(TIM2_BASE)
  if (Timer == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
#endif
#if defined(TIM3_BASE)
  if (Timer == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
#endif
#if defined(TIM4_BASE)
  if (Timer == TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
#endif
	
	_prescaler = prescaler-1;
	(_TIMER)->PSC = _prescaler;
	_ppr = pulsePerRevolution;
		
	//map inputs. 
	(_TIMER)->CCMR1 = 0x01 | (0x1<<8);//TIM_CCMR1_CC1S_INPUT_TI1 | TIMER_CCMR1_CC2S_INPUT_TI2;
	//Count mode
	(_TIMER)->SMCR = count_mode; //choose encoder 3, counting on both edges. 
	
	//set the interval used by the encoder.
	(_TIMER)->ARR = _ppr-1;//timer_set_reload(_TIMER, _ppr); //after each revolution, interrupt and increment/decrement. 

	//(_TIMER)->DIER |= TIM_DIER_UIE;
	//Go Timer, GO!!!!!!!
	(_TIMER)->CR1 |= TIM_CR1_CEN;//timer_resume(_TIMER);
	
};

float STM32Encoder::getAngle(unsigned char base){

	const float angle_base[3] = {360.0f, 6.2831f, 400.0f};

	if (base <= 2){
		return ((value()/(float)_ppr) * angle_base[base]);
	} else {
		return -900.0; //ridiculous, isn't it? 
	}
		
}

unsigned char STM32Encoder::getDirection(){
	return ((_TIMER)->CR1& TIM_CR1_DIR ? NEGATIVE:POSITIVE);//(TIM1->CR1 & TIM_CR1_DIR ? FORWARD : BACKWARD);
}

void STM32Encoder::Polarity(unsigned char pol) {
	if ( pol == INVERTED) {
		(_TIMER)->CCER |= (TIM_CCER_CC1P); //to invert the counting, only one of the inputs should be inverted.  
	}else{ //pol = NORMAL
		(_TIMER)->CCER &= ~( TIM_CCER_CC1P);
	}
}

void STM32Encoder::reset(){
	(_TIMER)->CNT=0;//timer_set_count(_TIMER, 0);
};

unsigned int STM32Encoder::value() {
	return (_TIMER)->CNT;//timer_get_count(_TIMER); 
};
	
void STM32Encoder::setPrescaler(unsigned int prescale){
    if (prescale == 0 ) prescale = 1;
    prescale--;//prescale for users should be between 1 and ~65000
	(_TIMER)->PSC=prescale;//timer_set_prescaler(_TIMER, prescale); //prescale for the chip is between 0 and ~65000
};

unsigned int STM32Encoder::getPrescaler(){
	return (_TIMER)->PSC;//timer_get_prescaler(_TIMER); 
};

void STM32Encoder::setPPR(unsigned int pulserPerRevolution){
	_ppr = pulserPerRevolution; 
	(_TIMER)->ARR=_ppr;//timer_set_reload(_TIMER, _ppr); //update counts.
};

unsigned int STM32Encoder::getPPR(){
	return _ppr;
};

extern "C"{
	
	void TIM1_IRQHandler(void)
  {
    if (TIM1->SR & TIM_SR_UIF) 
				TIM1->SR &= ~TIM_SR_UIF; // Clear the TIMx interrupt pending bit

		_isrptr_extia();		     
   
  }
	void TIM2_IRQHandler(void)
  {
    if (TIM2->SR & TIM_SR_UIF) 
				TIM2->SR &= ~TIM_SR_UIF; // Clear the TIMx interrupt pending bit

		_isrptr_extib();		     
   
  }
	void TIM3_IRQHandler(void)
  {
    if (TIM3->SR & TIM_SR_UIF) 
				TIM3->SR &= ~TIM_SR_UIF; // Clear the TIMx interrupt pending bit

		_isrptr_extic();		     
   
  }
	void TIM4_IRQHandler(void)
  {
    if (TIM4->SR & TIM_SR_UIF) 
				TIM4->SR &= ~TIM_SR_UIF; // Clear the TIMx interrupt pending bit

		_isrptr_extid();		     
   
  }
	
}
	
void STM32Encoder::attachInterrupt(void (*callback)(void )) {
IRQn_Type IRQn ;
	//attach interrupt
	//timer_attach_interrupt(_TIMER, TIMER_UPDATE_INTERRUPT, func);
	//configure EXTI_CR1/CR2 registers
	(_TIMER)->CR1 &= ~TIM_CR1_CEN;//timer_resume(_TIMER);
	if ((_TIMER)==TIM1) {			//configure interrupt on porta
		_isrptr_extia = callback;					//install user handler
		IRQn = TIM1_UP_IRQn;
	}

	if ((_TIMER)==TIM2) {			//configure interrupt on portb
		_isrptr_extib = callback;					//install user handler
		IRQn = TIM2_IRQn;
	}
	
	if ((_TIMER)==TIM3) {			//configure interrupt on portc
		_isrptr_extic = callback;					//install user handler
		IRQn = TIM3_IRQn;
	}
	
	if ((_TIMER)==TIM4) {			//configure interrupt on portd
		_isrptr_extid = callback;					//install user handler
		IRQn = TIM4_IRQn;
	}
	
	// configure Update interrupt
  HAL_NVIC_SetPriority(IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(IRQn);
	
	//Enable Interrupt	
	(_TIMER)->CR1  &= ~(TIM_CR1_UDIS);
	(_TIMER)->DIER |= TIM_DIER_UIE;
	//Go Timer, GO!!!!!!!
	(_TIMER)->CR1 |= TIM_CR1_CEN;//timer_resume(_TIMER);
	
	
}

void STM32Encoder::detachInterrupt() {
	(_TIMER)->CR1  |=TIM_CR1_UDIS;	
	
	void (*callback)(void) = empty_handler;	
	
	if ((_TIMER)==TIM1) {			//configure interrupt on porta
		_isrptr_extia = callback;					//install user handler
	}

	if ((_TIMER)==TIM2) {			//configure interrupt on portb
		_isrptr_extib = callback;					//install user handler
	}
	
	if ((_TIMER)==TIM3) {			//configure interrupt on portc
		_isrptr_extic = callback;					//install user handler
	}
	
	if ((_TIMER)==TIM4) {			//configure interrupt on portd
		_isrptr_extid = callback;					//install user handler
	}
	
	//timer_attach_interrupt(_TIMER, TIMER_UPDATE_INTERRUPT, NULL);	
}

void STM32Encoder::setFilter(unsigned char val) {

	val = val & 0xF; //clear the upper nibble.
	(_TIMER)->CCMR1 = ((_TIMER)->CCMR1 & 0x0F0F) | (val << 12) | (val <<4); 
}


/*
void STM32Encoder::timer_interrupt_handler(){
	if (Direction() == POSITIVE) _revolutions++;
	if (Direction() == NEGATIVE) _revolutions--;
}*/


/*
*  Remap Inputs
*  The table below indicates the pins that can be used with the remap method. 
*  TIMER3 is a special case as the partial remap can use completely different pins than no remap and full remap. 
*  All other timers either use the same pins (mind you the encoder only uses 2 channels) or don't have the possibility 
*  of partial remap. 
*  
*  Timer Remapping Table. 
*  TIMER1 - NORMAL - CH1 = PA8(D6), CH2 = PA9(D7) / FULL - CH1 = PE9, CH2 = PE11  
*  TIMER2 - NORMAL - CH1 = PA0(D2), CH2 = PA1(D3) / FULL - CH1 = PA15(D41), CH2 = PB3(D42) 
*  TIMER3 - NORMAL - CH1 = PA6(D12), CH2 = PA7(D11) / FULL - CH1 = PC6(D35), CH2 = PC7(D36) / PARTIAL - CH1 = PB4(D43), CH2 = PB5(D4)  
*  TIMER4 - NORMAL - CH1 = PB6(D5), CH2 = PB7(D9) / FULL - CH1 = PD12, CH2 = PD13
*/

/*
void STM32Encoder::remapInputs(unsigned char val){  //??? Is it worth it? 
//if this function is called, then we need to start the AFIO
	afio_init(); 
	unsigned char shift = 0; 

	switch(_TIMER) {
		case TIMER1: { //does this work???? 
			shift = 6;
			break;
		}
		case TIMER2: {
			shift = 8;
			break;
		}
		case TIMER3: {
			shift = 10; 
			break;
		}
		case TIMER4: {
			//hmmm... bit 12
			shift = 12;
			val = val % 1; //this is a bit, so we can only shift 1 or 0, not 3.
			break;
		}
		//TO DO: Support for Maple Native. 
	} //end switch
afio_remap( (val << shift));

}


}
*/
