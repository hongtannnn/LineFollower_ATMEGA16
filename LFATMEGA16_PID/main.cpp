#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

uint16_t thresholds[5] = {310, 270, 280, 310, 240};
int8_t weightValue[5]  = {-30, -10, 0, 10, 30};

uint8_t sensorD[5] = {0, 0, 0, 0, 0};
uint8_t sumSensor = 0;
int16_t sumWeight = 0;

float currPos = 0.0, cenPos = 0.0;
float eCurr = 0.0, ePrev = 0.0, last_eCurr = 0.0;
float dE = 0.0, eSum = 0.0, PID_val = 0.0;

int16_t base_speed = 130;
float Kp = 8.0, Ki = 0.0, Kd = 70.0;

uint8_t is_running = 0;
void UART_Init() {
	uint16_t ubrr = 103;
	
	// Set baud rate
	UBRRH = (unsigned char)(ubrr >> 8);
	UBRRL = (unsigned char)ubrr;

	UCSRB = (1 << TXEN);

	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void UART_Transmit(unsigned char data) {
	while (!(UCSRA & (1 << UDRE)));
	UDR = data;
}

void Send_Sensor_Data() {
	UART_Transmit('S'); UART_Transmit(':'); UART_Transmit(' ');
	for(uint8_t i = 0; i < 5; i++) {
		UART_Transmit(sensorD[i] ? '1' : '0');
		UART_Transmit(' ');
	}
	UART_Transmit('\r');
	UART_Transmit('\n');
}
int16_t constrain(int16_t val, int16_t min_val, int16_t max_val) {
	if (val < min_val) return min_val;
	if (val > max_val) return max_val;
	return val;
}

void ADC_Init() {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_Read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

void Hardware_Init() {
	DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4);
	
	DDRD |= (1 << PD4) | (1 << PD5);

	DDRD &= ~(1 << PD6);
	PORTD |= (1 << PD6);

	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
}

void Motor_Drive(int16_t left_pwm, int16_t right_pwm) {
	left_pwm = constrain(left_pwm, -255, 255);
	right_pwm = constrain(right_pwm, -255, 255);
	if (left_pwm >= 0) {
		PORTC &= ~(1 << PC0);
		PORTC |= (1 << PC1);
		} else {
		PORTC |= (1 << PC0);
		PORTC &= ~(1 << PC1);
	}
	OCR1A = abs(left_pwm);

	if (right_pwm >= 0) {
		PORTC |= (1 << PC2);
		PORTC &= ~(1 << PC3);
		} else {
		PORTC &= ~(1 << PC2);
		PORTC |= (1 << PC3);
	}
	OCR1B = abs(right_pwm);
}

void Read_Black_Line() {
	sumSensor = 0;
	sumWeight = 0;

	for (uint8_t i = 0; i < 5; i++) {
		uint16_t adc_val = ADC_Read(i);
		if (adc_val < thresholds[i]) {
			sensorD[i] = 0;
			} else {
			sensorD[i] = 1;
		}
		sumSensor += sensorD[i];
	}

	if (sumSensor > 0) {
		sumWeight = (sensorD[0] * weightValue[0] +
		sensorD[1] * weightValue[1] +
		sensorD[2] * weightValue[2] +
		sensorD[3] * weightValue[3] +
		sensorD[4] * weightValue[4]);
		
		currPos = (float)sumWeight / sumSensor;
		eCurr = cenPos - currPos;
	}
}

void Control_Logic() {
	if (sumSensor > 0) {
		last_eCurr = eCurr;
		dE = eCurr - ePrev;
		eSum += eCurr;
		
		if(eSum > 100) eSum = 100;
		if(eSum < -100) eSum = -100;
		
		PID_val = Kp * eCurr + Kd * dE + Ki * eSum;
		
		int16_t current_base_speed = base_speed;
		if (abs((int)eCurr) >= 15) {
			current_base_speed = base_speed / 2;
		}
		
		int16_t leftSpeed = current_base_speed - (int16_t)PID_val;
		int16_t rightSpeed = current_base_speed + (int16_t)PID_val;
		
		ePrev = eCurr;
		Motor_Drive(leftSpeed, rightSpeed);
		} else {
		if (last_eCurr < -10) {
			Motor_Drive(base_speed, -(base_speed * 0.3));
			} else if (last_eCurr > 10) {
			Motor_Drive(-(base_speed * 0.3), base_speed);
			} else {
			Motor_Drive(base_speed, base_speed);
		}
	}
}

int main(void) {
	MCUCSR |= (1 << JTD);
	MCUCSR |= (1 << JTD);

	ADC_Init();
	Hardware_Init();
	UART_Init();

	for(uint8_t i = 0; i < 6; i++) {
		PORTC ^= (1 << PC4);
		_delay_ms(150);
	}
	PORTC &= ~(1 << PC4);

	uint8_t send_counter = 0;
	while (1) {
		if (!(PIND & (1 << PD6))) {
			_delay_ms(50);
			if (!(PIND & (1 << PD6))) {
				is_running = !is_running;
				while (!(PIND & (1 << PD6)));
			}
		}
		Read_Black_Line();
		send_counter++;
		if (send_counter >= 10) {
			Send_Sensor_Data();
			send_counter = 0;
		}
		if (is_running == 1) {
			PORTC |= (1 << PC4);
			Control_Logic();
			} else {
			Motor_Drive(0, 0);
			PORTC &= ~(1 << PC4);
		}
		_delay_ms(5);
	}
}