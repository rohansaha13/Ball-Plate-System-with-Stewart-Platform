union BtoF
{
  byte b[16];
  float fval;
} u;

const int buffer_size=16;
byte buf[buffer_size];

void setup() {
  Serial.begin(115200,SERIAL_SN1);

}

void loop() {
  if(Serial.available()>0)
  {
    myVal= readFromMatlab();
    delay(0.1);
    writeToMatlab(myVal+5);
  }

}

float readFromMatlab()
{
  int reln = Serial.readBytesUntil("\r\n", buf, buffer_size);
  for(int i=0;i<buffer_size;i++)
  {
    u.b[i]= buf[i];
  }
  float output=u.fval;
  return output;
}

void writeToMatlab (float number)
{
  byte *b=(byte *) &number;
  Serial.write(b,4);
  Serial.write(13);
  Serial.write(10);
}