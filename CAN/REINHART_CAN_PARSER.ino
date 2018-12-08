 #include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

void setup()
{
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_250KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}
unsigned char CAN_ID = 0x94;
unsigned char message[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};
void loop()
{   
    unsigned char len = 0;
    unsigned char buf[8];
   // CAN.sendMsgBuf(CAN_ID, 0, 8, message);
   // delay(100);
 if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned long canId = CAN.getCanId();
        
        //Serial.println("-----------------------------");
        //Serial.print("Get data from ID: 0x");
        //Serial.println(canId, HEX);

if (canId == 0xA0){
            Serial.print("Module A Temperature: ");
            int aa = (buf[1]*256 + buf[0])/10;
            Serial.print (aa);
            Serial.println("\t");
            
            Serial.print("Module B Temperature: ");
            int ab = (buf[3]*256 + buf[2])/10;
            Serial.print (ab);
            Serial.println("\t");
            
            Serial.print("Module C Temperature: ");
            int ac = (buf[5]*256 + buf[4])/10;
            Serial.print (ac);
            Serial.println("\t");      
             
            Serial.print("Gate Driver Board Temperature: ");
            int ad = (buf[7]*256 + buf[6])/10;
            Serial.print (ad);
            Serial.println("\t");       
}

            if (canId == 0xA1){
            Serial.print("Control Board Temperature: ");
            int ba = (buf[1]*256 + buf[0])/10;
            Serial.print (ba);
            Serial.println("\t");
            
            Serial.print("RTD#1 Temperature: ");
            int bb = (buf[3]*256 + buf[2])/10;
            Serial.print (bb);
            Serial.println("\t");
            
            Serial.print("RTD#2 Temperature: ");
            int bc = (buf[5]*256 + buf[4])/10;
            Serial.print (bc);
            Serial.println("\t");      
             
            Serial.print("RTD#3 Temperature");
            int bd = (buf[7]*256 + buf[6])/10;
            Serial.print (bd);
            Serial.println("\t");       
            }

            if (canId == 0xA2){
            Serial.print("RTD#4 Temperature: ");
            int ca = (buf[1]*256 + buf[0])/10;
            Serial.print (ca);
            Serial.println("\t");
            
            Serial.print("RTD#5 Temperature: ");
            int cb = (buf[3]*256 + buf[2])/10;
            Serial.print (cb);
            Serial.println("\t");
            
            Serial.print("Motor Temperature: ");
            int cc = (buf[5]*256 + buf[4])/10;
            Serial.print (cc);
            Serial.println("\t");      
             
            Serial.print("Torque Shudder");
            int cd = (buf[7]*256 + buf[6])/10;
            Serial.print (cd);
            Serial.println("\t");       
            }

            if (canId == 0xA4){
            Serial.print("Digital Input #1: ");
            int ea = buf[0];
            Serial.print (ea);
            Serial.println("\t");
            
            Serial.print("Digital Input #2: ");
            int eb = buf[1];
            Serial.print (eb);
            Serial.println("\t");
            
            Serial.print("Digital Input #3: ");
            int ec = buf[2];
            Serial.print (ec);
            Serial.println("\t");      
             
            Serial.print("Digital Input #4:");
            int ed = buf[3];
            Serial.print (ed);
            Serial.println("\t");       
            
            Serial.print("Digital Input #5: ");
            int ee = buf[5];
            Serial.print (ee);
            Serial.println("\t");
            
            Serial.print("Digital Input #6: ");
            int ef = buf[5];
            Serial.print (ef);
            Serial.println("\t");
            
            Serial.print("Digital Input #7: ");
            int eg = buf[6];
            Serial.print (eg);
            Serial.println("\t");      
             
            Serial.print("Digital Input #8:");
            int eh = buf[7];
            Serial.print (eh);
            Serial.println("\t");       
            }

if (canId == 0xA5){
            Serial.print("Motor Angle: ");
            int fa = (buf[1]*256 + buf[0])/10;
            Serial.print (fa);
            Serial.println("\t");
            
            Serial.print("Motor Speed: ");
            int fb = buf[3]*256 + buf[2];
            Serial.print (fb);
            Serial.println("\t");
            
            Serial.print("Motor Output Frequency: ");
            int fc = (buf[5]*256 + buf[4])/10;
            Serial.print (fc);
            Serial.println("\t");      
             
            Serial.print("Delta Resolver Filtered");
            int fd = (buf[7]*256 + buf[6])/10;
            Serial.print (fd);
            Serial.println("\t");       
}

if (canId == 0xA6){
            Serial.print("Phase A Current: ");
            int ga = (buf[1]*256 + buf[0])/10;
            Serial.print (ga);
            Serial.println("\t");
            
            Serial.print("Phase B Current: ");
            int gb = (buf[3]*256 + buf[2])/10;
            Serial.print (gb);
            Serial.println("\t");
            
            Serial.print("Phase C Current: ");
            int gc = (buf[5]*256 + buf[4])/10;
            Serial.print (gc);
            Serial.println("\t");      
             
            Serial.print("DC Bus Current: ");
            int gd = (buf[7]*256 + buf[6])/10;
            Serial.print (gd);
            Serial.println("\t");       
}

if (canId == 0xA7){
            Serial.print("DC Bus Voltage: ");
            int ha = (buf[1]*256 + buf[0])/10;
            Serial.print (ha);
            Serial.println("\t");
            
            Serial.print("Output Voltage: ");
            int hb = (buf[3]*256 + buf[2])/10;
            Serial.print (hb);
            Serial.println("\t");
            
            Serial.print("VAB_Vd_Voltage: ");
            int hc = (buf[5]*256 + buf[4])/10;
            Serial.print (hc);
            Serial.println("\t");      
             
            Serial.print("VBC_Vq_Voltage: ");
            int hd = (buf[7]*256 + buf[6])/10;
            Serial.print (hd);
            Serial.println("\t");       
}

if (canId == 0xA8){
            Serial.print("Flux command: ");
            int ia = (buf[1]*256 + buf[0])/1000;
            Serial.print (ia);
            Serial.println("\t");
            
            Serial.print("Flux feedback: ");
            int ib = (buf[3]*256 + buf[2])/1000;
            Serial.print (ib);
            Serial.println("\t");
            
            Serial.print("Id feedback: ");
            int ic = (buf[5]*256 + buf[4])/10;
            Serial.print (ic);
            Serial.println("\t");      
             
            Serial.print("Iq feedback: ");
            int id = (buf[7]*256 + buf[6])/10;
            Serial.print (id);
            Serial.println("\t");       
}

if (canId == 0xA9){
            Serial.print("1.5V Reference voltage: ");
            int ja = (buf[1]*256 + buf[0])/100;
            Serial.print (ja);
            Serial.println("\t");
            
            Serial.print("2.5V Reference voltage: ");
            int jb = (buf[3]*256 + buf[2])/100;
            Serial.print (jb);
            Serial.println("\t");
            
            Serial.print("5.0V Reference voltage: ");
            int jc = (buf[5]*256 + buf[4])/100;
            Serial.print (jc);
            Serial.println("\t");      
             
            Serial.print("12V System voltage: ");
            int jd = (buf[7]*256 + buf[6])/100;
            Serial.print (jd);
            Serial.println("\t");       
}

if (canId == 0xAB){
            Serial.print("POSY Fault Lo : ");
            int la = (buf[1]*256 + buf[0])/10000;
            Serial.print (la);
            Serial.println("\t");
            
            Serial.print("POST Fault Hi: ");
            int lb = (buf[3]*256 + buf[2])/10000;
            Serial.print (lb);
            Serial.println("\t");
            
            Serial.print("Run Fault Lo: ");
            int lc = (buf[5]*256 + buf[4])/10000;
            Serial.print (lc);
            Serial.println("\t");      
             
            Serial.print("Run Fault Hi: ");
            int ld = (buf[7]*256 + buf[6])/10000;
            Serial.print (ld);
            Serial.println("\t");       
                 }

if (canId == 0xAC){
            Serial.print("Commanded Torque: ");
            int ma = (buf[1]*256 + buf[0])/10;
            Serial.print (ma);
            Serial.println("\t");
            
            Serial.print("Torque Feedback: ");
            int mb = (buf[3]*256 + buf[2])/10;
            Serial.print (mb);
            Serial.println("\t");
            
            Serial.print("Power on Time: ");
            int mc = (buf[7]*256*256*256 + buf[6]*256*256 + buf[5]*256 + buf[4])/10;
            Serial.print (mc);
            Serial.println("\t");      
                 }

if (canId == 0xAD){
            Serial.print("Modulation Index: ");
            int na = buf[1]*256 + buf[0];
            Serial.print (na);
            Serial.println("\t");
            
            Serial.print("Flux Weakening Output: ");
            int nb = (buf[3]*256 + buf[2])/10;
            Serial.print (nb);
            Serial.println("\t");
            
            Serial.print("Id command: ");
            int nc = (buf[5]*256 + buf[4])/10;
            Serial.print (nc);
            Serial.println("\t");      
             
            Serial.print("Iq command: ");
            int nd = (buf[7]*256 + buf[6])/10;
            Serial.print (nd);
            Serial.println("\t");       
                 }
        //Serial.println();

    }
}

// END FILE
