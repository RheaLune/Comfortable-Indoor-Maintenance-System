#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <compat/twi.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/signal.h>
#include <stdlib.h>
#include <math.h>

/////////////////////////////////////////////////////////////////////////

// Tera Term 테스트 코드
void uart1_init(void) {
    DDRD &= ~(_BV(2));
    DDRD |= _BV(3);
    UCSR1B = 0x00;
    UCSR1A = 0x00;
    UCSR1C = 0x06;
    UBRR1L = 0x33;
    UBRR1H = 0x00;
    UCSR1B = 0x08;
}

int uart1_putchar(char c) {
    UCSR1A = UCSR1A | 0x40;
    UDR1 = c;
    while(!(UCSR1A & 0x40));
}

int uart1_getchar(void) {
    char c;
    while(!(UCSR1A & 0x80));
    c = UDR1;
    uart1_putchar(c);
    return c;
}

void init_printf(void) {
    uart1_init();
    fdevopen(uart1_putchar, uart1_getchar);
}

void atmega128_init(void) {
    MCUCR = 0x00;

    DDRA=0x0; DDRB=0x0; DDRC=0x0; DDRD=0x0; DDRE=0x0; DDRF=0x0; DDRG=0x00;
    PORTA=0x0; PORTB=0x0; PORTC=0x0; PORTD=0x0; PORTE=0x0; PORTF=0x0; PORTG=0x0;

    init_printf();
}

/////////////////////////////////////////////////////////////////////////

// DHT-11 동작코드
#define INT8        unsigned char
#define INT16       unsigned int

INT8 temInt = 0, temDec = 0, humInt = 90, humDec = 0;

void getData(void) {
    INT8 bits[5];
    INT8 i, j;

    memset(bits, 0, sizeof(bits));  // 데이터 저장할 배열 초기화

    // reset port
    DDRF |= _BV(1);  // output
    PORTF |= _BV(1);  // high
    _delay_ms(100);

    // send request
    PORTF &= ~(_BV(1));  // low
    _delay_ms(18);
    PORTF |= _BV(1);  // high
    _delay_us(1);
    DDRF &= ~(_BV(1));  // input
    _delay_us(39);

    // check start condition 1
    if(PINF & (1<<1))
        return;
    _delay_us(80);
    // check start condition 2
    if(!(PINF & (1<<1)))
        return;
    _delay_us(80);

    // read the data
    for (j=0; j<5; j++) {  // read 5 byte
        INT8 result = 0;
        for(i = 0; i < 8; i++) {  // read every bit
            while(!(PINF & (1<<1)));  // wait for an high input
            _delay_us(30);
            if(PINF & (1<<1))  // if input is high after 30 us, get result
                result |= (1<<(7-i));
            while(PINF & (1<<1));  // wait until input get low
        }
        bits[j] = result;
    }

    // reset port
    DDRF |= _BV(1);  // output
    PORTF |= _BV(1);  // high
    _delay_ms(100);

    // check checksum
    if (bits[0] + bits[1] + bits[2] + bits[3] == bits[4]) {
        humInt = bits[0];
        humDec = bits[1];
        temInt = bits[2];
        temDec = bits[3];
    }
    return;
}

/////////////////////////////////////////////////////////////////////////

// DFPlayer 동작코드
#define BAUD 9600
#define U2X_S 2     // 2일때 비동기 2배속 모드, 1일때 비동기 일반 모드
#define MYUBRR ((F_CPU*U2X_S)/(16L*BAUD)-1)
//
#define MP3_VOLUME                      0x06  // 0~30
#define MP3_PLAYBACK_SOURCE             0x09  // 0 = U / 1 = TF
#define MP3_PLAYBACK_SOURCE_TF          1
#define MP3_SPECIFY_MP3_FOLDER          0X12  // 길어지므로 HEX로 표기
#define MP3_SET_REPEAT_CURRENT_TRACK    0X19  // 0 = ON

// frame format = [$S VER Len CMD Feedback para1 para2 checksum $O]
// flag: 고정, VER: 0xff고정, LEN: flag checksum제외 길이로 0x06고정, Feedback: feedback 받지 않을것으로 0고정
// CMD: 명령, para1 para2: 파라미터, checksum: 0xFFFF-(VER+Len+CMD+Feedback+para1+para2)
INT8 default_buffer[10] = {0x7E, 0xFF, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF}; // Default Buffer
volatile INT8 mp3_cmd_buf[10] = {0x7E, 0xFF, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF};
    
void USART1_Init(INT16 ubrr) {
    // Set baud rate
    UBRR1H = (INT8)(ubrr>>8);
    UBRR1L = (INT8)ubrr;
    // Enable U2X
    if(U2X_S == 2) UCSR1A |= _BV(U2X1);  // 비동기 2배속 모드
    else UCSR1A &= ~(_BV(U2X1));
    // Set frame format: 8data, 1stop bit
    UCSR1C &= ~(_BV(UMSEL1));  // asynch
    UCSR1C &= ~(_BV(USBS1));  // 1 Stop bit
    UCSR1C &= ~(_BV(UPM11));  // No parity
    UCSR1C &= ~(_BV(UPM10));
    UCSR1B &= ~(_BV(UCSZ12));  // 8-bit
    UCSR1C |= _BV(UCSZ11);
    UCSR1C |= _BV(UCSZ10);
    // Enable receiver and transmitter
    UCSR1B |= _BV(RXEN1);
    UCSR1B |= _BV(TXEN1);
}

void USART1_Transmit(char data) {
    // Wait for empty transmit buffer
    while ( !( UCSR1A & 0x20 ) );   // (1<<UDRE)
    // Put data into buffer, sends the data
    UDR1 = data;
}

INT16 MP3_checksum(void) {
    INT16 sum = 0;
    INT8 i;
    for (i = 1; i < 7; i++) {
        sum += mp3_cmd_buf[i];
    }
    return -sum;  // 0xFFFF-(VER+Len+CMD+Feedback+para1+para2) = ~(VER+Len+CMD+Feedback+para1+para2)
}

void MP3_send_cmd(INT8 cmd, INT8 high_arg, INT8 low_arg) {
    INT8 i;
    INT16 checksum;
    mp3_cmd_buf[3] = cmd;
    mp3_cmd_buf[5] = high_arg;
    mp3_cmd_buf[6] = low_arg;
    checksum = MP3_checksum();  // checksum calculate
    mp3_cmd_buf[7] = (INT8) ((checksum >> 8) & 0x00FF);
    mp3_cmd_buf[8] = (INT8) (checksum & 0x00FF);
    for(i = 0; i < 10; i++) {
        USART1_Transmit(mp3_cmd_buf[i]);
        mp3_cmd_buf[i] = default_buffer[i];  // 전송된 바이트는 바로 초기화
    }
}

void dfplayer_init(void) {
    MP3_send_cmd(MP3_PLAYBACK_SOURCE,0,MP3_PLAYBACK_SOURCE_TF);  // select microSD
    _delay_ms(10);
    MP3_send_cmd(MP3_VOLUME, 0, 25);  // volume 25
    _delay_ms(10);
}

/////////////////////////////////////////////////////////////////////////

// I2C LCD1602 동작코드
volatile char data[100], LCD, lcd_addr;

void twi_write(unsigned char address, unsigned char data) {
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //START

    while(!(TWCR & (1<<TWINT))); //TWINT flag 기다림 
    while((TWSR&0xF8) != 0x08);  //START 상태(08) 기다림  

    TWDR = (address << 1) & 0xfe;            // 7 bit address + 0(write)
    TWCR = (1<<TWINT)|(1<<TWEN); //전송 

    while(!(TWCR & (1<<TWINT))); 
    while((TWSR&0xF8) != 0x18);  //SLA+W ACK 상태(18) 기다림

    TWDR = data;                 //data 
    TWCR = (1<<TWINT)|(1<<TWEN); //전송  

    while(!(TWCR & (1<<TWINT)));
    while((TWSR&0xF8) != 0x28);   //Data ACK 상태(28) 기다림 

    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); //STOP
}

unsigned char twi_read(char address) {
    unsigned char data;

    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //START

    while(!(TWCR & (1<<TWINT))); //TWINT flag 기다림 
    while((TWSR&0xF8) != 0x08);  //START 상태(08) 기다림  

    TWDR = (address << 1) & 0xfe;            // 7 bit address + 0(write)
    TWCR = (1<<TWINT)|(1<<TWEN); //전송 

    while(!(TWCR & (1<<TWINT))); 
    while((TWSR&0xF8) != 0x18);  //SLA+W ACK 상태(18) 기다림

    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //Repeat START

    while(!(TWCR & (1<<TWINT)));
    while((TWSR&0xF8) != 0x10);  //Repeat START 상태(08) 기다림

    TWDR = (address << 1) | 0x01;            // 7 bit address + 1(read)
    TWCR = (1<<TWINT)|(1<<TWEN); //전송 

    while(!(TWCR & (1<<TWINT)));
    while((TWSR&0xF8) != 0x40);  //SLA+R ACK 상태(40) 기다림 

    TWCR = (1<<TWINT)|(1<<TWEN); //전송

    while(!(TWCR & (1<<TWINT)));
    while((TWSR&0xF8) != 0x58);  //ACK 상태(58) 기다림 

    data = TWDR; 

    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); //STOP

    return data; 
}

void LCD_command(INT8 command) {                /* write a command(instruction) to text LCD */
    LCD = (command & 0b11110000) | 0b00001100;  // MSB 4 bit, command mode, write mode, enable high
    twi_write(lcd_addr, LCD); 
    _delay_us(50);
    LCD &= ~(_BV(2));   // enable toggle
    twi_write(lcd_addr, LCD); 
    _delay_us(50);

    LCD = (command & 0b00001111) ;  
    LCD = LCD << 4;
    LCD = LCD | 0b00001100;     // LSB 4 bit, command mode, write mode, enable high
    twi_write(lcd_addr, LCD); 
    _delay_us(50);
    LCD &= ~(_BV(2));   // enable toggle
    twi_write(lcd_addr, LCD); 
    _delay_us(50);

    _delay_ms(1);           // LCD 제어를 위해 시간 지연 필요함
}

void LCD_data(INT8 data) {              /* display a character on text LCD */
    LCD = (data & 0b11110000) | 0b00001101;     // MSB 4 bit, data mode, write mode, enable high
    twi_write(lcd_addr, LCD); 
    _delay_us(50);
    LCD &= ~(_BV(2));   // enable toggle
    twi_write(lcd_addr, LCD); 
    _delay_us(50);

    LCD = (data & 0b00001111) ;     
    LCD = LCD << 4;
    LCD = LCD | 0b00001101;     // LSB 4 bit, data mode, write mode, enable high
    twi_write(lcd_addr, LCD); 
    _delay_us(50);
    LCD &= ~(_BV(2));   // enable toggle
    twi_write(lcd_addr, LCD); 
    _delay_us(50);

    _delay_ms(1);           // LCD 제어를 위해 시간 지연 필요함
}

void LCD_string(INT8 command, INT8 *string) {       /* display a string on LCD */
    LCD_command(command);               // start position of string

    while(*string != '\0') {            // display string 
        LCD_data(*string);
        string++;
    }
}

void LCD_nibble(int lcd_addr) {         // LCD 1602 initialize for nibble operation
    _delay_ms(20);
    twi_write(lcd_addr, 0b00111100);  //command mode, write mode, enable high   
    _delay_us(50);
    twi_write(lcd_addr, 0b00111000); // toggle e
    _delay_us(50);

    _delay_ms(4);
    twi_write(lcd_addr, 0b00111100);  //command mode, write mode, enable high    
    _delay_us(50);
    twi_write(lcd_addr, 0b00111000); // toggle e
    _delay_us(50);

    _delay_ms(1);
    twi_write(lcd_addr, 0b00111100);  //command mode, write mode, enable high   
    _delay_us(50);
    twi_write(lcd_addr, 0b00111000); // toggle e
    _delay_us(50);

    _delay_ms(4);
    twi_write(lcd_addr, 0b00101100); // command mode, write mode, enable high   
    _delay_us(50);
    twi_write(lcd_addr, 0b00101000); // toggle e
    _delay_us(50);
    twi_write(lcd_addr, 0b00101100); 
    _delay_us(50);
}

void LCD_initialize(void) {         /* initialize text LCD module */
    LCD_command(0x28);      // LCD function set(4 bit, 16x2 line, 5x7 dot)
    _delay_ms(2);
    LCD_command(0x0C);      // LCD display control(display ON, cursor OFF, blink OFF)
    _delay_ms(2);
    LCD_command(0x01);      // clear display
    _delay_ms(2);
    LCD_command(0x06);      // LCD entry mode set(increment, not shift)
    _delay_ms(2);
    LCD_command(0x02);      // Retrun Home
    _delay_ms(2);
    LCD_string(0x80+0x27, "                ");  // DDRAM에 저장된값 표기 방지
    _delay_ms(2);
    LCD_string(0x80+0x67, "                ");
    _delay_ms(2);
}

/////////////////////////////////////////////////////////////////////////

// 메인코드
#define normalMode      0
#define testMode        1
#define temValue        0
#define humValue        1
#define mp3Mode         0
#define machineMode     1
#define deactivate      0
#define activate        1
#define voiceOff        0
#define voiceOn         1
#define asmrOff         0
#define asmrOn          1
#define offCmdWait      2
#define onCmdWait       3
#define countable       0
#define countingWait    1
#define temStop         0
#define temRunning      1
#define humStop         0
#define humRunning      1
#define temTurning      27
#define humTurning      50

INT8 selectMode = normalMode, workSelect = mp3Mode, valueSelect = temValue;
INT8 playVoice = voiceOff, playAsmr = offCmdWait, Fan = offCmdWait, Humidifier = offCmdWait;
INT8 temCondition = temStop, humCondition = humStop, countCondition = countable;
INT8 tempTem = 0, remainderTem = 0, tempTemDec = 0;
INT8 tempHum = 0, remainderHum = 0;

// DHT11 PF1 (tem:0~50, hum:20~90)
// DFPlayer PD2 PD3 (rx tx)
// I2C LCD1602 PD0 PD1 (SCL SDA)
// Moter PB2(+V, INA) PB3(GND, INB) (voltage difference)
// Humidifier PC1
// INT switch PE4 PE5 PE6 PE7

int main(void) {
    // atmega128_init();

    // Fan, Humidifier port set Initialize
    DDRB |= _BV(2);
    DDRB |= _BV(3);
    PORTB = 0x00;
    DDRC |= _BV(1);
    PORTC = 0x00;

    // dfplayer Initialize
    USART1_Init(MYUBRR);  // 9600/8/n/1/n
    dfplayer_init();
    // I2C LCD1602 Initialize
    INT8 DHT11_buf[3];
    lcd_addr = 0x27;

    // Initial TWI Peripheral
    DDRD |= _BV(0);  // Set SCL. SDA output
    DDRD |= _BV(1);
    PORTD &= ~(_BV(0));  // Set SCL. SDA Low
    PORTD &= ~(_BV(1));
    TWSR = 0x01;  // Select Prescaler of 4
    TWBR = 18;  // SCL frequency = 16000000 / (16 + 2 * 18 * 4) = 100 khz

    LCD_nibble(lcd_addr);
    LCD_initialize();

    // interrupt Initialize
    DDRE &= 0b00001111;
    EICRB |= 0xff;
    EIMSK |= 0xf0;
    SREG |= _BV(7);
    
    // start music Play
    MP3_send_cmd(0x12, 0, 23);  // 시스템을 시작합니다
    _delay_ms(5000);
    MP3_send_cmd(0x12, 0, 20);  // 윈도우 부팅음
    _delay_ms(2500);


    while(1) {
        if(countCondition == countingWait)  // 온습도 수동조절시 버튼 연속 입력 방지
            countCondition = countable;
        // fan condition check
        if((temInt > temTurning) & (temCondition == temStop)) {
            temCondition = temRunning;
            if(playAsmr != onCmdWait)
                MP3_send_cmd(0x12, 0, 16);  // 온도가 높아 선풍기를 켭니다.
            Fan = activate;
        }
        else if((temInt < temTurning) & (temCondition == temRunning)) {
            temCondition = temStop;
            if(playAsmr != onCmdWait)
                MP3_send_cmd(0x12, 0, 17);  // 선풍기를 끕니다.
            Fan = deactivate;
        }
        // humidifier condition check
        if((humInt < humTurning) & (humCondition == humStop)) {
            humCondition = humRunning;
            if(playAsmr != onCmdWait)
                MP3_send_cmd(0x12, 0, 18);  // 습도가 낮아 가습기를 켭니다.
            Humidifier = activate;
        }
        else if((humInt > humTurning) & (humCondition == humRunning)) {
            humCondition = humStop;
            if(playAsmr != onCmdWait)
                MP3_send_cmd(0x12, 0, 19);  // 가습기를 끕니다.
            Humidifier = deactivate;
        }
        
        if((playVoice == voiceOn) & (playAsmr != onCmdWait)) {
            // variable check
            tempTem = temInt / 10;
            remainderTem = temInt % 10;
            tempTemDec = temDec;
            tempHum = humInt / 10;
            remainderHum = humInt % 10;
            // voice condition check part
            MP3_send_cmd(0x12, 0, 12); _delay_ms(1100);  // 현재 온도는
            if (remainderTem != 0) {
                if (tempTem == 1) { MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 십
                if (tempTem == 2) { MP3_send_cmd(0x12, 0, 2); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 이십
                if (tempTem == 3) { MP3_send_cmd(0x12, 0, 3); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 삼십
                if (tempTem == 4) { MP3_send_cmd(0x12, 0, 4); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 사십
                if (remainderTem == 1) { MP3_send_cmd(0x12, 0, 1); _delay_ms(800); }
                if (remainderTem == 2) { MP3_send_cmd(0x12, 0, 2); _delay_ms(800); }
                if (remainderTem == 3) { MP3_send_cmd(0x12, 0, 3); _delay_ms(800); }
                if (remainderTem == 4) { MP3_send_cmd(0x12, 0, 4); _delay_ms(800); }
                if (remainderTem == 5) { MP3_send_cmd(0x12, 0, 5); _delay_ms(800); }
                if (remainderTem == 6) { MP3_send_cmd(0x12, 0, 6); _delay_ms(800); }
                if (remainderTem == 7) { MP3_send_cmd(0x12, 0, 7); _delay_ms(800); }
                if (remainderTem == 8) { MP3_send_cmd(0x12, 0, 8); _delay_ms(800); }
                if (remainderTem == 9) { MP3_send_cmd(0x12, 0, 9); _delay_ms(800); }
            }
            else {
                if (tempTem == 0) { MP3_send_cmd(0x12, 0, 11); _delay_ms(800); }  // 영
                if (tempTem == 1) { MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 십
                if (tempTem == 2) { MP3_send_cmd(0x12, 0, 2); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 이십
                if (tempTem == 3) { MP3_send_cmd(0x12, 0, 3); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 삼십
                if (tempTem == 4) { MP3_send_cmd(0x12, 0, 4); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 사십
                if (tempTem == 5) { MP3_send_cmd(0x12, 0, 5); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 오십
            }
            MP3_send_cmd(0x12, 0, 25); _delay_ms(800); // 점
            if (tempTemDec == 1) { MP3_send_cmd(0x12, 0, 1); _delay_ms(800); }
            if (tempTemDec == 2) { MP3_send_cmd(0x12, 0, 2); _delay_ms(800); }
            if (tempTemDec == 3) { MP3_send_cmd(0x12, 0, 3); _delay_ms(800); }
            if (tempTemDec == 4) { MP3_send_cmd(0x12, 0, 4); _delay_ms(800); }
            if (tempTemDec == 5) { MP3_send_cmd(0x12, 0, 5); _delay_ms(800); }
            if (tempTemDec == 6) { MP3_send_cmd(0x12, 0, 6); _delay_ms(800); }
            if (tempTemDec == 7) { MP3_send_cmd(0x12, 0, 7); _delay_ms(800); }
            if (tempTemDec == 8) { MP3_send_cmd(0x12, 0, 8); _delay_ms(800); }
            if (tempTemDec == 9) { MP3_send_cmd(0x12, 0, 9); _delay_ms(800); }
            if (tempTemDec == 0) { MP3_send_cmd(0x12, 0, 11); _delay_ms(800); }  // 영
            MP3_send_cmd(0x12, 0, 13); _delay_ms(2000);  // 도 입니다

            MP3_send_cmd(0x12, 0, 14); _delay_ms(1100);  // 현재 습도는
            if (remainderHum != 0) {
                if (tempHum == 2) { MP3_send_cmd(0x12, 0, 2); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 이십
                if (tempHum == 3) { MP3_send_cmd(0x12, 0, 3); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 삼십
                if (tempHum == 4) { MP3_send_cmd(0x12, 0, 4); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 사십
                if (tempHum == 5) { MP3_send_cmd(0x12, 0, 5); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 오십
                if (tempHum == 6) { MP3_send_cmd(0x12, 0, 6); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 육십
                if (tempHum == 7) { MP3_send_cmd(0x12, 0, 7); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 칠십
                if (tempHum == 8) { MP3_send_cmd(0x12, 0, 8); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 팔십
                if (remainderHum == 1){ MP3_send_cmd(0x12, 0, 1); _delay_ms(800); }
                if (remainderHum == 2) { MP3_send_cmd(0x12, 0, 2); _delay_ms(800); }
                if (remainderHum == 3) { MP3_send_cmd(0x12, 0, 3); _delay_ms(800); }
                if (remainderHum == 4) { MP3_send_cmd(0x12, 0, 4); _delay_ms(800); }
                if (remainderHum == 5) { MP3_send_cmd(0x12, 0, 5); _delay_ms(800); }
                if (remainderHum == 6) { MP3_send_cmd(0x12, 0, 6); _delay_ms(800); }
                if (remainderHum == 7) { MP3_send_cmd(0x12, 0, 7); _delay_ms(800); }
                if (remainderHum == 8) { MP3_send_cmd(0x12, 0, 8); _delay_ms(800); }
                if (remainderHum == 9) { MP3_send_cmd(0x12, 0, 9); _delay_ms(800); }
            }
            else {
                if (tempHum == 2) { MP3_send_cmd(0x12, 0, 2); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 이십
                if (tempHum == 3) { MP3_send_cmd(0x12, 0, 3); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 삼십
                if (tempHum == 4) { MP3_send_cmd(0x12, 0, 4); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 사십
                if (tempHum == 5) { MP3_send_cmd(0x12, 0, 5); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 오십
                if (tempHum == 6) { MP3_send_cmd(0x12, 0, 6); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 육십
                if (tempHum == 7) { MP3_send_cmd(0x12, 0, 7); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 칠십
                if (tempHum == 8) { MP3_send_cmd(0x12, 0, 8); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 팔십
                if (tempHum == 9) { MP3_send_cmd(0x12, 0, 9); _delay_ms(800); MP3_send_cmd(0x12, 0, 10); _delay_ms(800); }  // 구십
            }
            MP3_send_cmd(0x12, 0, 15);  // 퍼센트 입니다
            playVoice = voiceOff;
        }
        else if((playVoice == voiceOn) & (playAsmr == onCmdWait))
            playVoice = voiceOff;

        if(playAsmr == asmrOn) {
            playAsmr = onCmdWait;
            // asmr on
            MP3_send_cmd(0x12, 0, 21);  // 좋은 꿈 꾸세요
            _delay_ms(2000);
            MP3_send_cmd(0x12, 0, 22);  // asmr 음원 play
            MP3_send_cmd(MP3_SET_REPEAT_CURRENT_TRACK, 0, 0);  // asmr 반복
        }

        if(playAsmr == asmrOff) {
            playAsmr = offCmdWait;
            // asmr off
            MP3_send_cmd(0x16, 0, 0);  // stop
            MP3_send_cmd(0x12, 0, 24);  // asmr을 종료합니다.
        }

        if(Fan == activate) {
            Fan = onCmdWait;
            PORTB |= _BV(2);  // fan on
        }

        if(Fan == deactivate) {
            Fan = offCmdWait;
            PORTB &= ~(_BV(2));  // fan off
        }

        if(Humidifier == activate) {
            Humidifier = onCmdWait;
            PORTC |= _BV(1);  // humidifier on
        }

        if(Humidifier == deactivate) {
            Humidifier = offCmdWait;
            PORTC &= ~(_BV(1));  // humidifier off
        }

        if(selectMode == normalMode)
            getData();  // run DHT11

        // data display
        LCD_string(0x80, "TEM:");
        itoa(temInt, DHT11_buf, 10);
        LCD_string(0x80+0x04, DHT11_buf);
        LCD_data('.');
        itoa(temDec, DHT11_buf, 10);
        LCD_string(0x80+0x07, DHT11_buf);
        LCD_string(0x80+0x08, "  MODE");

        LCD_string(0x80+0x40, "HUM:");
        itoa(humInt, DHT11_buf, 10);
        LCD_string(0x80+0x44, DHT11_buf);
        LCD_data('.');
        itoa(humDec, DHT11_buf, 10);
        LCD_string(0x80+0x47, DHT11_buf);
        if(selectMode == normalMode) LCD_string(0x80+0x48, "  ACT   ");
        if(selectMode == testMode) LCD_string(0x80+0x48, "  TEST  ");

        _delay_ms(1000);
    }

    return (1);
}

/////////////////////////////////////////////////////////////////////////

// ISR 동작코드
// 인터럽트는 최대한 빠르게 빠지기위해 변수로만 컨트롤
// mode select button
ISR(INT4_vect) {
    if(selectMode == normalMode) {
        selectMode = testMode;
    }
    else {
        selectMode = normalMode;
    }
}

// play voice/Fan on, value increase button
ISR(INT5_vect) {
    if(selectMode == normalMode) {
        if(workSelect == mp3Mode) {
            if(playVoice == voiceOff) {
                playVoice = voiceOn;
            }
        }
        else {
            if(Fan == offCmdWait) {
                Fan = activate;
            }
            else if(Fan == onCmdWait) {
                Fan = deactivate;
            }
        }
    }
    else {
        if(valueSelect == temValue) {
            if((temInt < 50) & (countCondition == countable)) {
                countCondition = countingWait;
                temInt++;
            }
        }
        else {
            if((humInt < 90) & (countCondition == countable)) {
                countCondition = countingWait;
                humInt++;
            }
        }
    }
}

// play asmr/Humidifier on, value decrease button
ISR(INT6_vect) {
    if(selectMode == normalMode) {
        if(workSelect == mp3Mode) {
            if(playAsmr == offCmdWait) {
                playAsmr = asmrOn;
            }
            else if(playAsmr == onCmdWait) {
                playAsmr = asmrOff;
            }
        }
        else {
            if(Humidifier == offCmdWait) {
                Humidifier = activate;
            }
            else if(Humidifier == onCmdWait) {
                Humidifier = deactivate;
            }
        }
    }
    else {
        if(valueSelect == temValue) {
            if((temInt > 0) & (countCondition == countable)) {
                countCondition = countingWait;
                temInt--;
            }
        }
        else {
            if((humInt > 20) & (countCondition == countable)) {
                countCondition = countingWait;
                humInt--;
            }
        }
    }
}

// work select, value select button
ISR(INT7_vect) {
    if(selectMode == normalMode) {
        if(workSelect == mp3Mode) {
            workSelect = machineMode;
        }
        else {
            workSelect = mp3Mode;
        }
    }
    else {
        if(valueSelect == temValue) {
            valueSelect = humValue;
        }
        else {
            valueSelect = temValue;
        }
    }
}