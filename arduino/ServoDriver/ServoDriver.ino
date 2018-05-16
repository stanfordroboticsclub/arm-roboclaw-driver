
#define ADDRESS 131
#define M1DUTYCOMMAND 32
#define READVERSION 21

unsigned char buffer[10];
int i = 0;

int duration1 = 1500;
int duration2 = 1500;
int target = 90;
int val = 90;

//Calculates CRC16 of nBytes of data in byte array message
uint16_t crc16(unsigned char *packet, int nBytes) {
  uint16_t crc = 0;
  for (int byte = 0; byte < nBytes; byte++) {
    crc = crc ^ ((unsigned int)packet[byte] << 8);
    for (unsigned char bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      }else{
        crc=crc<<1;
      }
    }
  }
  return crc;
}

char version_string[] = "AATeensy imitating Roboclaw\n";

void setup() {
 pinMode(2,OUTPUT);
 pinMode(3,OUTPUT);

 version_string[0] = ADDRESS;
 version_string[1] = READVERSION;
 
 Serial1.begin(115200);
  Serial.begin(115200);
}



void loop() {
  while( Serial1.available() ){
    buffer[i] = Serial1.read();
    i++;
    if(i>9){
      while( Serial1.available() ) Serial1.read();
      i=0;
      break;
    }}
    

    if(i>0){

       if(i<2) i=2;
       uint16_t ch = crc16(buffer, i-2);
//       unsigned int ch = crc16(buffer, 4);
       
       if(buffer[0] == ADDRESS &&
          buffer[1] == M1DUTYCOMMAND &&
          ( ((uint16_t)buffer[4])<<8 | ((uint16_t)buffer[5])) == ch)
          {
            target = buffer[2]<<8 | buffer[3];
            Serial1.write(255);
          }  
          else if(buffer[0] == ADDRESS &&
          buffer[1] == READVERSION &&
          ( ((uint16_t)buffer[2])<<8 | ((uint16_t)buffer[4])) == ch)
          
          {

            
            
            uint16_t verify = crc16( (unsigned char*)version_string, strlen(version_string)+1);

            Serial.println(verify);
            Serial1.print(version_string+2);
            Serial1.write(0);
            Serial1.write((char)(verify >> 8));
            Serial1.write((char)verify);
          }


          
          i = 0;
    }
    


  if(val!=target){
      if(val<target) val++;
      if(val>target) val--;
       duration1 = map(val,0,180,500,2500);
       duration2 = map(val,0,180,2500,500);
  }

  //Doing the pulse generation manually leads to less jitter then using a library
  digitalWrite(2,HIGH);
  delayMicroseconds(duration1);
  digitalWrite(2,LOW);

  digitalWrite(3,HIGH);
  delayMicroseconds(duration2);
  digitalWrite(3,LOW);
}
