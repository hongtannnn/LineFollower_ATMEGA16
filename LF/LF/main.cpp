#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

uint16_t thresholds[5] = {310, 270, 280, 310, 240};

uint8_t sensorD[5] = {0, 0, 0, 0, 0};
uint8_t sumSensor = 0;

int16_t base_speed = 130;

int8_t last_dir = 0;

uint8_t is_running = 0;

void UART_Init() {
	uint16_t ubrr = 103;

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

	for (uint8_t i = 0; i < 5; i++) {
		uint16_t adc_val = ADC_Read(i);
		if (adc_val < thresholds[i]) {
			sensorD[i] = 0; // Tr?ng
			} else {
			sensorD[i] = 1; // ?en
		}
		sumSensor += sensorD[i];
	}
}

void Control_Logic() {
	// 1. Xe ? gi?a Line
	if (sensorD[2] == 1 && sensorD[1] == 0 && sensorD[3] == 0) {
		Motor_Drive(base_speed, base_speed);
		last_dir = 0;
	}
	// 2. Xe l?ch nh? sang PH?I (v?ch ?en c?n vào c?m bi?n S1 bên trái)
	else if (sensorD[1] == 1 && sensorD[0] == 0) {
		Motor_Drive(base_speed * 0.5, base_speed + 20); // Phanh bánh trái, t?ng bánh ph?i ?? r? trái
		last_dir = -1;
	}
	// 3. Xe l?ch nhi?u sang PH?I (v?ch ?en c?n vào c?m bi?n S0 ngoài cùng bên trái)
	else if (sensorD[0] == 1) {
		Motor_Drive(-50, base_speed + 30); // Bánh trái ch?y lùi nh? ?? cua g?t
		last_dir = -2;
	}
	// 4. Xe l?ch nh? sang TRÁI (v?ch ?en c?n vào c?m bi?n S3 bên ph?i)
	else if (sensorD[3] == 1 && sensorD[4] == 0) {
		Motor_Drive(base_speed + 20, base_speed * 0.5); // Phanh bánh ph?i, t?ng bánh trái ?? r? ph?i
		last_dir = 1;
	}
	// 5. Xe l?ch nhi?u sang TRÁI (v?ch ?en c?n vào c?m bi?n S4 ngoài cùng bên ph?i)
	else if (sensorD[4] == 1) {
		Motor_Drive(base_speed + 30, -50); // Bánh ph?i ch?y lùi nh? ?? cua g?t
		last_dir = 2;
	}
	// 6. X? lý khi m?t Line hoàn toàn (sumSensor == 0)
	else if (sumSensor == 0) {
		if (last_dir == -1 || last_dir == -2) {
			Motor_Drive(-60, base_speed);
		}
		else if (last_dir == 1 || last_dir == 2) {
			Motor_Drive(base_speed, -60);
		}
		else {
			Motor_Drive(base_speed, base_speed);
		}
	}
}

int main(void) {
	// 1. T?T JTAG (Gi?i phóng PC2, PC3, PC4)
	MCUCSR |= (1 << JTD);
	MCUCSR |= (1 << JTD);

	// 2. Kh?i t?o
	ADC_Init();
	Hardware_Init();
	UART_Init();

	// Nháy LED
	for(uint8_t i = 0; i < 6; i++) {
		PORTC ^= (1 << PC4);
		_delay_ms(150);
	}
	PORTC &= ~(1 << PC4);

	// Bi?n ??m ?? chia nh? t?n s? g?i Bluetooth
	uint8_t send_counter = 0;

	while (1) {

		if (!(PIND & (1 << PD6))) {
			_delay_ms(50); // Debounce)
			if (!(PIND & (1 << PD6))) {
				
				is_running = !is_running; //
				
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