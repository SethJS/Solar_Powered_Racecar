void send_data(unsigned char data);

void setup()
{
pinMode(4,OUTPUT);
pinMode(5,OUTPUT);
pinMode(6,OUTPUT);
pinMode(7,OUTPUT);
Serial.begin(9600);
}
void loop()
{
unsigned char i=0;
for(i=0;i<3;i++)
{
send_data(i);
Serial.print("i=");
Serial.println(i,DEC);
delay(180);
send_data(0);
delay(180);
}
}
//====================================
void send_data(unsigned char data)
{
digitalWrite(4,(data&0x01));
digitalWrite(5,(data&0x02));
digitalWrite(6,(data&0x04));
digitalWrite(7,(data&0x08));
}











/*unsigned int i=17;
void send_data(unsigned int data);


void setup()
{
pinMode(7,OUTPUT);
Serial.begin(9600);
}
void loop()
{
send_data(i);
delay(20);
}
//====================================
void send_data(unsigned int data)
{
digitalWrite(7,(data));
Serial.print("data=");
Serial.println(data,DEC);
}*/
