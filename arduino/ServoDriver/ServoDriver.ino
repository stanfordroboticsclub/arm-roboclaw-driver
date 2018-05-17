
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


while (Serial1.available() ){
      unsigned char address = Serial1.read();
      Serial.print("Got address: "); Serial.println(address);

      if(address == ADDRESS){
        unsigned char command = Serial1.read();

        Serial.print("Got command: "); Serial.println(command);


        if(command == M1DUTYCOMMAND){

              if(!Serial1.available()) break;
              unsigned char high_val = Serial1.read();
              
              if(!Serial1.available()) break;
              unsigned char low_val = Serial1.read();

              if(!Serial1.available()) break;
              unsigned char high_crc = Serial1.read();
              
              if(!Serial1.available()) break;
              unsigned char low_crc = Serial1.read();

              buffer[0] = address;
              buffer[1] = command;
              buffer[2] = high_val;
              buffer[3] = low_val;

              uint16_t ch = crc16(buffer, 4);
              if(( ( ( ((uint16_t)high_crc)<<8) )  | ((uint16_t)low_crc) ) == ch){
                target = high_val<<8 | low_val;
              }

        }else if(command == READVERSION){

      
              if(!Serial1.available()) break;
              unsigned char high_crc = Serial1.read();
              
              if(!Serial1.available()) break;
              unsigned char low_crc = Serial1.read();

              Serial.print("Got high: "); Serial.println(high_crc);
              Serial.print("Got low : "); Serial.println(low_crc);

              buffer[0] = address;
              buffer[1] = command;

              uint16_t ch = crc16(buffer, 2);

              if(( ( ( ((uint16_t)high_crc)<<8) )  | ((uint16_t)low_crc) ) == ch){
                
                    uint16_t verify = crc16( (unsigned char*)version_string, strlen(version_string)+1);
        
                    Serial.println(verify);
                    Serial1.print(version_string+2);
                    Serial1.write(0);
                    Serial1.write((char)(verify >> 8));
                    Serial1.write((char)verify);
                    Serial.println("version command");
                
              }
              
        }

      }  
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
