bool Enable_cable_MANDO=0; bool Enable_cable_NCL=0; bool Enable_cable_NCM=0; bool Enable_cable_CIM=0; bool Enable_cable_ROBOT=0;
uint8_t PIN_DB9_CP_MANDO_DIR=51; uint8_t PIN_DB9_CP_MANDO_PUL=49; uint8_t PIN_DB9_AUX2_IN=47; uint8_t PIN_DB9_AUX1_IN=45;
uint8_t PIN_CABLE_IN_NCM_CONETADO=43; uint8_t PIN_CABLE_IN_NCL_CONETADO=41; uint8_t PIN_RIEL_IN_DATO_0_AUX=39; uint8_t PIN_RIEL_IN_DATO_1_AUX=37;
uint8_t PIN_RIEL_IN_ACTIVACION=35; uint8_t PIN_PARO_EMERG1_IN=33; uint8_t PIN_CABLE_CIM_CONECTADO=31; uint8_t PIN_ROBOT_IN_DATO_0=29;
uint8_t PIN_ROBOT_IN_DATO_1=27;  uint8_t PIN_ROBOT_IN_ACTIVACION=25; uint8_t PIN_CABLE_IN_ROBOT_CONECTADO=23;

uint8_t PIN_AUX_OUT_CP1=48;
uint8_t PIN_AUX_OUT_CP2=46;
uint8_t PIN_AUX_OUT_CP3=44;

uint8_t PIN_FMS_OUT_ENABLE=13;
uint8_t PIN_CIM_OUT_ONLINE_LED=12;
uint8_t PIN_OUT_MOV_TO_SP_MANUAL=11;//  PIN_RIEL_OUT_A2=11;
uint8_t PIN_RIEL_MANUAL_AUTO=10;//      PIN_RIEL_OUT_A1=10;
uint8_t PIN_RIEL_IZQUIERDA=9;//         PIN_RIEL_OUT_A0=9;
uint8_t PIN_RIEL_DERECHA=8;//           PIN_RIEL_OUT_MOV_TO_SP_AUTO=8;
uint8_t PIN_ROBOT_OUT_DATO_0=7;
uint8_t PIN_ROBOT_OUT_DATO_1=6;
uint8_t PIN_ROBOT_OUT_DATO_2=5;
uint8_t PIN_ROBOT_OUT_DATO_ACTIVACION=4;
uint8_t FIN_CARRERA_1_RIEL=3;           //PIN_RIEL_IN_DATO_0_AUX=39; // PIN_AUX_IN_CP1****da 2 V
uint8_t FIN_CARRERA_2_RIEL=2;           //PIN_RIEL_IN_DATO_1_AUX=37; // PIN_AUX_IN_CP2
uint8_t ENCODER_RIEL=20;                //PIN_RIEL_IN_ACTIVACION=35; // PIN_AUX_IN_CP3

struct MANDO_VAR { //Motor_Errores1
bool EP0; bool EP1; bool EP2; bool ET0; bool ET1; bool ET2; bool EE0; bool EE1; bool EE2; bool ER0; 
bool ER1; bool ER2; bool EA0; bool EA1; bool EA2; bool EA3; bool ES0; bool ES1; bool ES3; bool ES2; 
bool EV0; bool EV1; bool EV2; bool EC0; bool EC1; bool EC2; bool EM0; bool EM1; bool EM2;
bool rev; uint8_t enviando_comando_tipo;
bool enable_enviado_peticion_completo; uint8_t peticion_recibida;
bool enviado_trabajo_completo; uint8_t trabajar_con_recibida;
bool enviado_estado_completo; uint8_t estado_recibida; 
bool enviado_error_completo; uint8_t error_recibida;
bool enviado_accion_completo; uint8_t accion_recibida; uint8_t accion_num_recibida;
bool enviado_velocidad_completo; String velocidad_acu_recibida; uint8_t velocidad_recibida;
bool recibido_nuevo_comando;
bool enviado_valvulas_completo; uint8_t valvulas_recibida;
bool enviado_cable_completo;
bool enviado_modo_completo;
};

MANDO_VAR MANDO={0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,
0,0,0,0,0,0,0,  0,0,0,0,0,  0,0,"",0,
0,
0,0, 0,0 };

struct NCL_VAR { //Motor_Errores1
bool EE0; bool EE1; bool EE2; bool ER0; bool ER1; bool ER2; bool ES0; bool ES1; 
bool ES3; bool ES2; bool EV3; bool EV1; bool EV2; bool EJ0; bool EJ1; bool EJ2;
bool rev; uint8_t enviando_comando_tipo;
bool enviado_estado_completo; uint8_t estado_recibida; 
bool enviado_error_completo; uint8_t error_recibida;
bool recibido_nuevo_comando;
bool enviado_valvulas_completo; uint8_t valvulas_recibida;
bool enviado_eje_completo; uint8_t eje_recibida;
bool enviado_velocidad_completo; String velocidad_acu_recibida; uint8_t velocidad_recibida;
};

NCL_VAR NCL={0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,0,
0,0,0,0,0,  0,0,0,0,0,  0,0,"",0};

struct NCM_VAR { //Motor_Errores1
bool EE0; bool EE1; bool EE2; bool ER0; bool ER1; bool ER2; bool ES0; bool ES1; 
bool ES3; bool ES2; bool EV3; bool EV1; bool EV2; bool EJ0; bool EJ1; bool EJ2;
bool rev; uint8_t enviando_comando_tipo;
bool enviado_estado_completo; uint8_t estado_recibida; 
bool enviado_error_completo; uint8_t error_recibida;
bool recibido_nuevo_comando;
bool enviado_valvulas_completo; uint8_t valvulas_recibida;
bool enviado_eje_completo; uint8_t eje_recibida;
bool enviado_velocidad_completo; String velocidad_acu_recibida; uint8_t velocidad_recibida;
};

NCM_VAR NCM={0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,0,
0,0,0,0,0,  0,0,0,0,0,  0,0,"",0};
/////////////////////////////////////////////////////////////////////////****************
uint8_t trabajando_con_act=0;
bool    modo_CP_act=0;
uint8_t estado_CP_act=0;
int     error_CP_act=0;
int     estado_cables_CP_act=0;

bool    modo_NCM_act=0;
uint8_t estado_NCM_act=0;
int     error_NCM_act=0;
uint8_t valvulas_NCM_act=0;
uint8_t velocidad_NCM_act=0;
uint8_t eje_NCM_act=0;

bool    modo_NCL_act=0;
uint8_t estado_NCL_act=0;
int     error_NCL_act=0;
uint8_t valvulas_NCL_act=0;
uint8_t velocidad_NCL_act=0;
uint8_t eje_NCL_act=0;

bool    modo_MANDO_act=0;
uint8_t estado_MANDO_act=0;
int     error_MANDO_act=0;
uint8_t valvulas_MANDO_act=0;
uint8_t eje_MANDO_act=0;

uint8_t estado_ROBOT_act=0;
uint8_t estado_RIEL_act=0;
/////////////////////*******************

//Peticion de mover ejes NCL o NCM
bool    enable_mover_ejes_manualmente=0;
uint8_t ejes_para_mover_manualmente=0;

//Peticion de cambio de velocidad NCL o NCM
uint8_t velocidad_MANDO_act=0;
bool    enable_cambiar_velocidad=0;

//Peticion de movimiento riel
bool    enable_riel_movimiento_manual=0;

bool    enable_riel_movimiento_a_posicion_manualmente=0;
uint8_t posicion_a_mover_riel_manual=0;

//Peticion de movimiento robot
bool    enable_robot_movimiento_a_posicion_manualmente=0;
uint8_t posicion_a_mover_robot_manual=0;

//Peticion de Activacion de Valvulas
bool    enable_activacion_valvulas=0;
uint8_t bin_recibido_act_valvulas=0;

//Peticion de Cambio de modo
bool    enable_cambiar_modo=0;
bool    cambiar_a_modo=0;

bool enable_enviar_cambio_a_estaciones=0;
bool enable_apagar_al_desactivar_modo_manual_RIEL=0;
bool enable_apagar_al_desactivar_modo_manual_RIEL_posicion=0;
bool enable_apagar_al_desactivar_modo_manual_ROBOT=0;
bool enable_notificar_al_cambiar_trabajar_con_NCM=0;
bool enable_notificar_al_cambiar_trabajar_con_NCL=0;

void setup() {
  pinMode(48, OUTPUT);  //AUX_OUT
  pinMode(46, OUTPUT);  //AUX_OUT
  pinMode(44, OUTPUT);  //AUX_OUT
  pinMode(13, OUTPUT);  //FMS_OUT_ENABLE
  pinMode(12, OUTPUT);  //CIM_OUT_ONLINE_LED
  pinMode(11, OUTPUT);  //RIEL_OUT_A2
  pinMode(10, OUTPUT);  //RIEL_OUT_A1
  pinMode(9,  OUTPUT);  //RIEL_OUT_A0
  pinMode(8,  OUTPUT);  //RIEL_OUT_SP
  pinMode(7,  OUTPUT);  //ROBOT_OUT_0
  pinMode(6,  OUTPUT);  //ROBOT_OUT_1
  pinMode(5,  OUTPUT);  //ROBOT_OUT_2
  pinMode(4,  OUTPUT);  //ROBOT_OUT_3
  pinMode(3,  INPUT);  //IN3
  pinMode(2,  INPUT);  //IN2
  pinMode(20, INPUT); //IN1 ME DA 2V
  pinMode(23, INPUT); // CABLE ROBOT
  pinMode(25, INPUT); // ROBOT IA
  pinMode(27, INPUT); // ROBOT I1
  pinMode(29, INPUT); // ROBOT I0
  pinMode(31, INPUT); //CABLE CIM
  pinMode(33, INPUT); //PE
  pinMode(35, INPUT); //IA
  pinMode(37, INPUT); //I1
  pinMode(39, INPUT); //I0
  pinMode(41, INPUT); //CABLE NCL
  pinMode(43, INPUT); //CABLE NCM
  pinMode(45, INPUT); //X1 - AUX2 DB9
  pinMode(47, INPUT); //X2 - AUX2 DB9
  pinMode(49, INPUT); //PUL
  pinMode(51, INPUT); //DIR
  reset_acciones();
  Serial.begin (9600);
  Serial1.begin(9600); //NCL
  Serial2.begin(9600);  //NCM
  Serial3.begin(9600);  //Mando
  delay(500);
  Serial.println(F("Bienvenido FMS2101 - Control Principal"));
}

void loop() {
//      if(digitalRead(Enable_cable_MANDO)){
    Control_Principal();
      if (Serial1.available() > 0) { //NCL_1
        Recepcion_Dato_NCL(); //NOOOO OLVIDAR CANBIAR EL Serial1.read();
      }
      if (Serial2.available() > 0) { //NCM_2
        Recepcion_Dato_NCM(); //NOOOO OLVIDAR CANBIAR EL Serial2.read();
      }
      if (Serial3.available() > 0) { //MANDO_3
        Recepcion_Dato_MANDO(); //NOOOO OLVIDAR CANBIAR EL Serial3.read();
      }
      if(NCL.recibido_nuevo_comando){
        NCL.recibido_nuevo_comando=0;
        Respuesta_a_comandos_NCL();
        imprimir_debug_NCL();
      }
      if(NCM.recibido_nuevo_comando){
        NCM.recibido_nuevo_comando=0;
        Respuesta_a_comandos_NCM();
      }
      if(MANDO.recibido_nuevo_comando){
        MANDO.recibido_nuevo_comando=0;
        Respuesta_a_comandos_MANDO();
        imprimir_debug_MANDO();
      }
  //}
}
void enviar_a_NCL(String comand){ //NCL_1
  Serial1.println(comand);
}
void enviar_a_NCM(String comand){ //NCM_2
  Serial2.println(comand);
}
void enviar_a_MANDO(String comand){ //MANDO_3
  Serial3.println(comand);
}
      
void imprimir_debug_MANDO(){
Serial.print("estado_MANDO_act:               "); Serial.println(estado_MANDO_act);
Serial.print("error_MANDO_act:                "); Serial.println(error_MANDO_act);
Serial.print("enable_activacion_valvulas:     "); Serial.println(enable_activacion_valvulas);
Serial.print("bin_recibido_act_valvulas:      "); Serial.println(bin_recibido_act_valvulas);
Serial.print("enable_cambiar_modo:            "); Serial.println(enable_cambiar_modo);
Serial.print("cambiar_a_modo:                 "); Serial.println(cambiar_a_modo);
Serial.print("enable_robot.._manual:          "); Serial.println(enable_robot_movimiento_a_posicion_manualmente);
Serial.print("posicion_a_mover_robot_manual:  "); Serial.println(posicion_a_mover_robot_manual);
Serial.print("enable_riel_movimiento_manual:  "); Serial.println(enable_riel_movimiento_manual);
Serial.print("enable_riel_...n_manualmente:   "); Serial.println(enable_riel_movimiento_a_posicion_manualmente);
Serial.print("posicion_a_mover_riel_manual:   "); Serial.println(posicion_a_mover_riel_manual);
Serial.print("enable_mover_ejes_manualmente:  "); Serial.println(enable_mover_ejes_manualmente);
Serial.print("ejes_para_mover_manualmente:    "); Serial.println(ejes_para_mover_manualmente);
Serial.print("enable_cambiar_velocidad:       "); Serial.println(enable_cambiar_velocidad);
Serial.print("velocidad_MANDO_act:            "); Serial.println(velocidad_MANDO_act);
Serial.println();
}
void imprimir_debug_NCL(){
Serial.print("estado_NCL_act:     "); Serial.println(estado_NCL_act);
Serial.print("error_NCL_act:      "); Serial.println(error_NCL_act);
Serial.print("velocidad_NCL_act:  "); Serial.println(velocidad_NCL_act);
Serial.print("valvulas_NCL_act:   "); Serial.println(valvulas_NCL_act);
Serial.print("eje_NCL_act:        "); Serial.println(eje_NCL_act);
}

////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////
////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////
////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////
void Control_Principal(){
  Verificar_Cables_Conectados();
  Verificar_Acciones_A_Realizar();
}

void Verificar_Cables_Conectados(){
  estado_cables_CP_act=0;
  if(!digitalRead(PIN_DB9_AUX2_IN)){ //Cable de MANDO
        Enable_cable_MANDO=1;
        estado_cables_CP_act|=0b00000001; //Añade un 1
  }else{
        Enable_cable_MANDO=0;
        estado_MANDO_act=6;
        estado_cables_CP_act&=0b11111110; //Añade un 0
  }
  if(!digitalRead(PIN_CABLE_IN_NCL_CONETADO)){ //Cable de NCL
        estado_cables_CP_act|=0b00000010;//Añade un 1
        Enable_cable_NCL=1;
  }else{
        estado_cables_CP_act&=0b11111101; //Añade un 0    
        Enable_cable_NCL=0;
        estado_NCL_act=6;
  }
  if(!digitalRead(PIN_CABLE_IN_NCM_CONETADO)){ //Cable de NCM
        estado_cables_CP_act|=0b00000100;//Añade un 1
        Enable_cable_NCM=1;
  }else{
        estado_cables_CP_act&=0b11111011; //Añade un 0    
        Enable_cable_NCM=0;
        estado_NCM_act=6;
  }
  if(!digitalRead(PIN_CABLE_IN_ROBOT_CONECTADO)){ //Cable de CIM
        estado_cables_CP_act|=0b00010000;//Añade un 1
        Enable_cable_ROBOT=1;
  }else{
        estado_cables_CP_act&=0b11101111; //Añade un 0    
        Enable_cable_ROBOT=0;
        estado_ROBOT_act=6;
  }
  
  if(!digitalRead(PIN_CABLE_CIM_CONECTADO)){ //Cable de CIM  PIN_CABLE_CIM_CONECTADO
        estado_cables_CP_act|=0b00001000;//Añade un 1
        Enable_cable_CIM=1;
  }else{
        estado_cables_CP_act&=0b11110111; //Añade un 0
        Enable_cable_CIM=0;
  }
  Enable_cable_MANDO=1;
  Enable_cable_NCL=1;
  Enable_cable_NCM=1;
  Enable_cable_CIM=1;
  Enable_cable_ROBOT=1;
  ////////////////////////////////////////////////////////////////
  if(estado_cables_CP_act!=0b00011111){
    estado_CP_act=2;
  }else{
    estado_CP_act=0;
  }
  if(!digitalRead(PIN_PARO_EMERG1_IN) && Enable_cable_CIM){
    estado_CP_act=5;
    digitalWrite(PIN_FMS_OUT_ENABLE,0);
    digitalWrite(PIN_CIM_OUT_ONLINE_LED,0);
  }else{
    digitalWrite(PIN_FMS_OUT_ENABLE,1);
    digitalWrite(PIN_CIM_OUT_ONLINE_LED,1);
  }
  if(Enable_cable_ROBOT){
    if(!digitalRead(PIN_ROBOT_IN_DATO_0) && !digitalRead(PIN_ROBOT_IN_DATO_1)){
      estado_ROBOT_act=0; // Robot_Preparado y en Home
    }
    if(digitalRead(PIN_ROBOT_IN_DATO_0) && !digitalRead(PIN_ROBOT_IN_DATO_1)){
      estado_ROBOT_act=2; // Robot_con_errores
    }
    if(!digitalRead(PIN_ROBOT_IN_DATO_0) && digitalRead(PIN_ROBOT_IN_DATO_1)){
      estado_ROBOT_act=4; // Robot_Trabajando
    }
    if(digitalRead(PIN_ROBOT_IN_DATO_0) && digitalRead(PIN_ROBOT_IN_DATO_1)){
      estado_ROBOT_act=1; // Robot_Ocupado
    }
  }
}


void Verificar_Acciones_A_Realizar(){

  if(Enable_cable_MANDO && trabajando_con_act!=2 && trabajando_con_act!=3){
      if(enable_cambiar_modo){ //mover ejes
        enable_cambiar_modo=0;
        if(cambiar_a_modo){
            modo_CP_act=1;
        }else{
            modo_CP_act=0;
        }
      }
  }
  if(trabajando_con_act!=2 && enable_notificar_al_cambiar_trabajar_con_NCM && Enable_cable_NCM){
    enable_notificar_al_cambiar_trabajar_con_NCM=0;
    enviar_a_NCM("AM0");
  }
  if(trabajando_con_act!=3 && enable_notificar_al_cambiar_trabajar_con_NCL && Enable_cable_NCL){
    enable_notificar_al_cambiar_trabajar_con_NCL=0;
    enviar_a_NCL("AL0");
  }
  if(trabajando_con_act==2 && Enable_cable_NCM){
      if(enable_mover_ejes_manualmente){ //mover ejes
        enable_mover_ejes_manualmente=0;
        enviar_a_NCM("AM"+String(ejes_para_mover_manualmente, DEC));
        enable_notificar_al_cambiar_trabajar_con_NCM=1; //ENVIA UN AM0 CUANDO CAMBIE DE TRABAJO CON
      }
      if(enable_cambiar_velocidad){ //Cambiar velocidad
        enable_cambiar_velocidad=0;
        enviar_a_NCM("S"+String(velocidad_MANDO_act, DEC));
      }
      if(enable_cambiar_modo){
        enable_cambiar_modo=0;
        if(cambiar_a_modo){
            enviar_a_NCM("AC1");
        }else{
            enviar_a_NCM("AC0");
        }
      }
  }
  if(trabajando_con_act==3 && Enable_cable_NCL){//posicion_a_mover_riel_manual
      if(enable_mover_ejes_manualmente){ //mover ejes
        enable_mover_ejes_manualmente=0;
        enviar_a_NCL("AL"+String(ejes_para_mover_manualmente, DEC));
        enable_notificar_al_cambiar_trabajar_con_NCL=1; //ENVIA UN AL0 CUANDO CAMBIE DE TRABAJO CON
      }
      if(enable_cambiar_velocidad){ //Cambiar velocidad
        enable_cambiar_velocidad=0;
        enviar_a_NCL("S"+String(velocidad_MANDO_act, DEC));
      }
      if(enable_cambiar_modo){
        enable_cambiar_modo=0;
        if(cambiar_a_modo){
            enviar_a_NCL("AC1");
        }else{
            enviar_a_NCL("AC0");
        }
      }
  }

  if(trabajando_con_act==4 && Enable_cable_CIM){
        if(enable_riel_movimiento_manual && !digitalRead(PIN_DB9_CP_MANDO_PUL)){ //mover ejes
          if(digitalRead(PIN_DB9_CP_MANDO_DIR)){
            digitalWrite(PIN_RIEL_DERECHA,0);
            digitalWrite(PIN_RIEL_IZQUIERDA,1);
          }else{
            digitalWrite(PIN_RIEL_IZQUIERDA,0);
            digitalWrite(PIN_RIEL_DERECHA,1);
          }
          enable_apagar_al_desactivar_modo_manual_RIEL=1;
          digitalWrite(PIN_RIEL_MANUAL_AUTO,1);
        }
        if(enable_riel_movimiento_a_posicion_manualmente){ //Cambiar velocidad
          if(!digitalRead(PIN_DB9_CP_MANDO_PUL) && digitalRead(PIN_DB9_CP_MANDO_DIR)){
            digitalWrite(PIN_RIEL_DERECHA,0);
            digitalWrite(PIN_RIEL_IZQUIERDA,1);
          }else{
            if(!digitalRead(PIN_DB9_CP_MANDO_PUL) && !digitalRead(PIN_DB9_CP_MANDO_DIR)){
              digitalWrite(PIN_RIEL_IZQUIERDA,0);
              digitalWrite(PIN_RIEL_DERECHA,1);
            }
          }
          digitalWrite(PIN_RIEL_MANUAL_AUTO,1);
          enable_apagar_al_desactivar_modo_manual_RIEL_posicion=1;
        }
//          if(digitalRead(FIN_CARRERA_1_RIEL)){ //PIN_AUX_IN_CP3=20;
//          }
//          if(digitalRead(FIN_CARRERA_2_RIEL)){ //PIN_AUX_IN_CP3=20;
//          }  
//          ENCODER_RIEL=20;      //PIN_AUX_IN_CP1=20;
  }

  if(trabajando_con_act==5 && Enable_cable_ROBOT){
      if(enable_robot_movimiento_a_posicion_manualmente){ //Cambiar velocidad
        posicion_a_mover_robot_manual=1;
        if((posicion_a_mover_robot_manual & 0b00000100)==0b00000100){
          digitalWrite(PIN_ROBOT_OUT_DATO_2,1);
        }else{
          digitalWrite(PIN_ROBOT_OUT_DATO_2,0);
        }
        if((posicion_a_mover_robot_manual & 0b00000010)==0b00000010){
          digitalWrite(PIN_ROBOT_OUT_DATO_1,1);
        }else{
          digitalWrite(PIN_ROBOT_OUT_DATO_1,0);
        }
        if((posicion_a_mover_robot_manual & 0b00000001)==0b00000001){
          digitalWrite(PIN_ROBOT_OUT_DATO_0,1);
        }else{
          digitalWrite(PIN_ROBOT_OUT_DATO_0,0);
        }
        if(digitalRead(PIN_DB9_CP_MANDO_PUL)){
          digitalWrite(PIN_ROBOT_OUT_DATO_ACTIVACION,1);
          enable_apagar_al_desactivar_modo_manual_ROBOT=1;
        }
      }
  }
  
  if(trabajando_con_act!=4){
    if(enable_riel_movimiento_manual || enable_riel_movimiento_a_posicion_manualmente){
      enable_riel_movimiento_manual=0;
      enable_riel_movimiento_a_posicion_manualmente=0;
    }
  }
  if(digitalRead(PIN_DB9_CP_MANDO_PUL )){
    digitalWrite(PIN_RIEL_IZQUIERDA,0);
    digitalWrite(PIN_RIEL_DERECHA,0);
    digitalWrite(PIN_ROBOT_OUT_DATO_ACTIVACION,0);
  }
  if(enable_apagar_al_desactivar_modo_manual_RIEL && !enable_riel_movimiento_manual){
    enable_apagar_al_desactivar_modo_manual_RIEL=0;
    digitalWrite(PIN_RIEL_IZQUIERDA,0);
    digitalWrite(PIN_RIEL_DERECHA,0);
    digitalWrite(PIN_RIEL_MANUAL_AUTO,0);
  }
  if(enable_apagar_al_desactivar_modo_manual_RIEL_posicion && !enable_riel_movimiento_a_posicion_manualmente){
    enable_apagar_al_desactivar_modo_manual_RIEL_posicion=0;
    digitalWrite(PIN_RIEL_IZQUIERDA,0);
    digitalWrite(PIN_RIEL_DERECHA,0);
    digitalWrite(PIN_RIEL_MANUAL_AUTO,0);
  }
  if(enable_apagar_al_desactivar_modo_manual_ROBOT && !enable_robot_movimiento_a_posicion_manualmente){
    enable_apagar_al_desactivar_modo_manual_ROBOT=0;
    digitalWrite(PIN_ROBOT_OUT_DATO_ACTIVACION,0);
    digitalWrite(PIN_RIEL_MANUAL_AUTO,0);
  }
}

////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////
////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////
////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////////CONTROL PRINCIPAL/////

////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////
////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////
////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////
void Recepcion_Dato_MANDO(){
  char c1 = Serial3.read();
  MANDO.rev=1;
  if(MANDO.enviando_comando_tipo==0){
    Buscar_comandos_MANDO(c1);
  }
  if(MANDO.rev) grafcet_MANDO(c1);
}
void Buscar_comandos_MANDO(char c){ // Todos los comandos que puede enviar el MANDO
  if(MANDO.enviando_comando_tipo==0){
    MANDO.rev=0;
    switch (c) {
      case 'P': //MANDO.peticion_recibida
        MANDO.enviando_comando_tipo=1;
        MANDO.EP0=0;
        MANDO.EP1=1;
      break;
      case 'T': //trabajo
        MANDO.enviando_comando_tipo=2;
        MANDO.ET0=0;
        MANDO.ET1=1;
      break;
      case 'E': // MANDO.estado_recibida
        MANDO.enviando_comando_tipo=3;
        MANDO.EE0=0;
        MANDO.EE1=1;
      break;
      case 'R': //error
        MANDO.enviando_comando_tipo=4;
        MANDO.ER0=0;
        MANDO.ER1=1;
      break;
      case 'A': //MANDO.accion_recibida
        MANDO.enviando_comando_tipo=5;
        MANDO.EA0=0;
        MANDO.EA1=1;
      break;
      case 'S': //velocidad
        MANDO.enviando_comando_tipo=6;
        MANDO.ES0=0;
        MANDO.ES1=1;
      break;
//      case 'V': //valvulas
//        MANDO.enviando_comando_tipo=7;
//        MANDO.EV0=0;
//        MANDO.EV1=1;
//      break;
//      case 'C': //cable
//        MANDO.enviando_comando_tipo=8;
//        MANDO.EC0=0;
//        MANDO.EC1=1;
//      break;
//      case 'M': //modo
//        MANDO.enviando_comando_tipo=9;
//        MANDO.EM0=0;
//        MANDO.EM1=1;
//      break;
      default:
        MANDO.rev=1;
        Serial.print("Error No se encuentra mando");
        Serial.println(c);
      break;
    }
  }
}
//////////////////////////////////////////////////////////////////////////////GRACET COMUNICACION//////////////////////////////////////////////////////////////
void grafcet_MANDO(char c){
    switch (MANDO.enviando_comando_tipo) {
    case 1:
      grafcet_peticiones_MANDO(c); //depende con quien se trabaje
    break;
    case 2:
      grafcet_trabajos_MANDO(c);  //extrusivo mando
    break;
    case 3:
      grafcet_estados_MANDO(c);   //guarda el estado del MANDO
    break;
    case 4:
      grafcet_errores_MANDO(c);   //guarda el estado del MANDO
    break;
    case 5:
      grafcet_acciones_MANDO(c);  //depende con quien se trabaje
    break;
    case 6:
      grafcet_velocidad_MANDO(c); // solo funciona cuando se trabaja con NCL y NCM
    break;
//    case 7:
//      grafcet_valvulas_MANDO(c);
//    break;
//    case 8:
//      grafcet_cable_MANDO(c);
//    break;
//    case 9:
//      grafcet_modo_MANDO(c);
//    break;
    default:
    break;
  }
}

void grafcet_peticiones_MANDO(char c){
  if(MANDO.EP1){
    MANDO.EP1=0;
    MANDO.EP2=1;
    MANDO.rev=0;
    switch (c) {
      case 'E': //Estado_actual
        MANDO.peticion_recibida=1;
      break;
      case 'M': //Modo_actual
        MANDO.peticion_recibida=2;
      break;
      case 'A': //Alertas
        MANDO.peticion_recibida=3;
      break;
      case 'R': //Errores
        MANDO.peticion_recibida=4;
      break;
      case 'V': //Estado_Electrovalvulas
        MANDO.peticion_recibida=5;
      break;
      case 'C': //Cables desconectados
        MANDO.peticion_recibida=6;
      break;            
      default:
        MANDO.rev=1;
        MANDO.EP0=1;
        MANDO.EP1=0;
        MANDO.EP2=0;
        MANDO.enviando_comando_tipo=0;
        Serial.print("Error_P: ");
        Serial.println(c);
      break;
    }
  }
  if(MANDO.EP2 && MANDO.rev){
    MANDO.EP0=1;
    MANDO.EP2=0;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      MANDO.enable_enviado_peticion_completo=1;
      MANDO.recibido_nuevo_comando=1;
      Serial.print("MANDO.peticion_recibida: ");
      Serial.println(MANDO.peticion_recibida);
      MANDO.rev=0;
    }
  }
}

void grafcet_trabajos_MANDO(char c){
  if(MANDO.ET1){
    MANDO.ET1=0;
    MANDO.ET2=1;
    MANDO.rev=0;
    switch (c) {
      case '0': //CP
        MANDO.trabajar_con_recibida=0;
      break;
      case '1':  //CP
        MANDO.trabajar_con_recibida=1;
      break;
      case '2': //NCM
        MANDO.trabajar_con_recibida=2;
      break;
      case '3': //NCL
        MANDO.trabajar_con_recibida=3;
      break;
      case '4': //Riel
        MANDO.trabajar_con_recibida=4;
      break;
      case '5': //Robot
        MANDO.trabajar_con_recibida=5;
      break;        
      default:
        MANDO.rev=1;
        MANDO.ET0=1;
        MANDO.ET1=0;
        MANDO.ET2=0;
        MANDO.enviando_comando_tipo=0;
        Serial.print("Error_T: ");
        Serial.println(c);
      break;
    }
  }
  if(MANDO.ET2 && MANDO.rev){
    MANDO.ET0=1;
    MANDO.ET2=0;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      MANDO.enviado_trabajo_completo=1;
      MANDO.recibido_nuevo_comando=1;
      Serial.print("Trabajo: ");
      Serial.println(MANDO.trabajar_con_recibida);
      MANDO.rev=0;
    }
  }
}
void grafcet_estados_MANDO(char c){
    if(MANDO.EE1){
      MANDO.EE1=0;
      MANDO.EE2=1;
      MANDO.rev=0;
      switch (c) {
        case '0': //Preparado
          MANDO.estado_recibida=0;
        break;
        case '1'://Ocupado
          MANDO.estado_recibida=1;
        break;
        case '2'://Con errores o Alertas
          MANDO.estado_recibida=2;
        break;
        case '3'://deshabilitado
          MANDO.estado_recibida=3;
        break;
        case '4'://Trabajando
          MANDO.estado_recibida=4;
        break;
        case '5'://Paro de Emergencia
          MANDO.estado_recibida=5;
        break;
        case '6'://Desconectado
          MANDO.estado_recibida=6;
        break;    
        default:
          MANDO.rev=1;
          MANDO.EE0=1;
          MANDO.EE1=0;
          MANDO.EE2=0;
          MANDO.enviando_comando_tipo=0;
          Serial.print("Error_E: ");
          Serial.println(c);
        break;
      }
    }
    if(MANDO.EE2 && MANDO.rev){
      MANDO.EE0=1;
      MANDO.EE2=0;
      MANDO.enviando_comando_tipo=0;
      if(c==10 || c==13){
        MANDO.enviado_estado_completo=1;
        MANDO.recibido_nuevo_comando=1;
        Serial.print("MANDO.estado_recibida: ");
        Serial.println(MANDO.estado_recibida);
        MANDO.rev=0;
      }
    }
}
void grafcet_errores_MANDO(char c){
  if(MANDO.ER1){
    MANDO.ER1=0;
    MANDO.ER2=1;
    MANDO.rev=0;
    switch (c) {
      case '0': // no error
        MANDO.error_recibida=0;
      break;
      case '1': // Joystick_error
        MANDO.error_recibida=1;
      break;
      default:
        MANDO.rev=1;
        MANDO.ER0=1;
        MANDO.ER1=0;
        MANDO.ER2=0;
        MANDO.enviando_comando_tipo=0;
        Serial.print("Error_E: ");
        Serial.println(c);
      break;
    }
  }
  if(MANDO.ER2 && MANDO.rev){
    MANDO.ER0=1;
    MANDO.ER2=0;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      MANDO.enviado_error_completo=1;
      MANDO.recibido_nuevo_comando=1;
      Serial.print("error: ");
      Serial.println(MANDO.error_recibida);
      MANDO.rev=0;
    }
  }
}
void grafcet_acciones_MANDO(char c){
  if(MANDO.EA1){
    MANDO.EA1=0;
    MANDO.EA2=1;
    MANDO.rev=0;
    switch (c) {
        case '1': //Spindler
          MANDO.accion_recibida=1;
        break;
        case '2': //Cooler
          MANDO.accion_recibida=2;
        break;
        case '3': //Piston de Agarre
          MANDO.accion_recibida=3;
        break;
        case '4': //Puerta
          MANDO.accion_recibida=4;
        break;
        case 'C': //Cambiar Modo Manual/Automatico
          MANDO.accion_recibida=5;
        break;
        //////////////////////////////////
        case 'R': //Robot ir a posicion #
          MANDO.accion_recibida=10;
        break;
        case 'I': //Riel Modo Manual o automatico
          MANDO.accion_recibida=11;
        break;
        case 'J': //Riel ir a posicion #
          MANDO.accion_recibida=12;
        break;
        case 'M': // Mover manualmente eje # de NCM
          MANDO.accion_recibida=13;
        break;
        case 'L': // Mover manualmente eje # de NCL
          MANDO.accion_recibida=14;
        break;            
        default:
          MANDO.rev=1;
          MANDO.EA0=1;
          MANDO.EA2=0;
          MANDO.enviando_comando_tipo=0;
          Serial.print("Error_A: ");
          Serial.println(c);
        break;
    }
  }
  if(MANDO.EA2 && MANDO.rev){
    MANDO.EA3=1;
    MANDO.EA2=0;
    MANDO.rev=0;
    if(isdigit(c)){
      if(MANDO.accion_recibida<10){
        switch (c) {
          case '0':
            MANDO.accion_num_recibida=0;
          break;
          case '1':
            MANDO.accion_num_recibida=1;
          break;
          default:
            MANDO.rev=1;
            MANDO.EA3=0;
            MANDO.EA0=1;
            MANDO.accion_recibida=0;
            MANDO.enviando_comando_tipo=0;
            Serial.print("Error_Discreto: ");
            Serial.println(c);
          break;
        }
      }else{
        MANDO.accion_num_recibida=String(c).toInt();
      }
    }else{
      MANDO.rev=1;
      MANDO.EA3=0;
      MANDO.EA0=1;
      MANDO.accion_recibida=0;
      MANDO.enviando_comando_tipo=0;
      Serial.print("Error_subA: ");
      Serial.println(c);
    }
  }
  if(MANDO.EA3 && MANDO.rev){
    MANDO.EA3=0;
    MANDO.EA0=1;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      MANDO.enviado_accion_completo=1;
      MANDO.recibido_nuevo_comando=1;
      Serial.print("MANDO.accion_recibida: ");
      Serial.print(MANDO.accion_recibida);
      Serial.print(" y ");
      Serial.println(MANDO.accion_num_recibida);
      MANDO.rev=0;
    }else{
      Serial.print("falto_enter: ");
      Serial.println(c);
    }
  }
}

void grafcet_velocidad_MANDO(char c){
  if(MANDO.ES1){
    MANDO.ES1=0;
    MANDO.ES2=1;
    MANDO.rev=0;
    if(isdigit(c)){
      MANDO.velocidad_acu_recibida+=c;
    }else{
      MANDO.rev=1;
      MANDO.ES0=1;
      MANDO.ES2=0;
      MANDO.velocidad_acu_recibida="";
      MANDO.enviando_comando_tipo=0;
      Serial.print("Error_Speed1: ");
      Serial.println(c);
    }
  }
  if(MANDO.ES2 && MANDO.rev){
    MANDO.ES2=0;
    MANDO.ES3=1;
    MANDO.rev=0;
    if(isdigit(c)){
      MANDO.velocidad_acu_recibida+=c;
      MANDO.velocidad_recibida=String(MANDO.velocidad_acu_recibida).toInt();
      MANDO.velocidad_acu_recibida="";
    }else{
      MANDO.rev=1;
      MANDO.ES0=1;
      MANDO.ES3=0;
      MANDO.velocidad_acu_recibida="";
      MANDO.enviando_comando_tipo=0;
      Serial.print("Error_Speed2: ");
      Serial.println(c);
    }
  }
  if(MANDO.ES3 && MANDO.rev){
    MANDO.ES3=0;
    MANDO.ES0=1;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      MANDO.enviado_velocidad_completo=1;
      MANDO.recibido_nuevo_comando=1;
      Serial.print("MANDO.velocidad_recibida");
      Serial.println(MANDO.velocidad_recibida);
      MANDO.rev=0;
    }else{
      Serial.print("falto_enter: ");
      Serial.println(c);
    }
  }
}
//void grafcet_valvulas_MANDO(char c){
//  if(MANDO.EV1){
//    MANDO.EV1=0;
//    MANDO.EV2=1;
//    MANDO.rev=0;
//    if(isdigit(c)){
//      MANDO.valvulas_recibida+=String(c).toInt();
//    }else{
//      MANDO.rev=1;
//      MANDO.EV0=1;
//      MANDO.EV2=0;
//      MANDO.enviando_comando_tipo=0;
//      Serial.print("Error_valvulas: ");
//      Serial.println(c);
//    }
//  }
//  if(MANDO.EV2 && MANDO.rev){
//    MANDO.EV2=0;
//    MANDO.EV0=1;
//    MANDO.enviando_comando_tipo=0;
//    if(c==10 || c==13){
//      MANDO.enviado_valvulas_completo=1;
//      MANDO.recibido_nuevo_comando=1;
//      Serial.print("MANDO.valvulas_recibida ");
//      Serial.println(MANDO.valvulas_recibida);
//      MANDO.rev=0;
//    }else{
//      Serial.print("falto_enter: ");
//      Serial.println(c);
//    }
//  }
//}
//void grafcet_cable_MANDO(char c){
//    if(MANDO.EC1){
//      
//    }
//}
//void grafcet_modo_MANDO(char c){
//    if(MANDO.EM1){
//      
//    }
//}
////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////
////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////
////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////////COMUNICACION_MANDO////

////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////
////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////
////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////
    void reset_acciones(){
        //Peticion de mover ejes NCL o NCM
            enable_mover_ejes_manualmente=0;
            ejes_para_mover_manualmente=0;
        //Peticion de cambio de velocidad NCL o NCM
            velocidad_MANDO_act=0;
            enable_cambiar_velocidad=0;
        //Peticion de movimiento riel
            enable_riel_movimiento_manual=0;
            enable_riel_movimiento_a_posicion_manualmente=0;
            posicion_a_mover_riel_manual=0;
        //Peticion de movimiento robot
            enable_robot_movimiento_a_posicion_manualmente=0;
            posicion_a_mover_robot_manual=0;
        //Peticion de Activacion de Valvulas
            enable_activacion_valvulas=0;
            bin_recibido_act_valvulas=0;
        //Peticion de Cambio de modo
            enable_cambiar_modo=0;
            cambiar_a_modo=0;
    }

    void Respuesta_a_comandos_MANDO(){
        if(MANDO.enviado_trabajo_completo){
          MANDO.enviado_trabajo_completo=0;
          trabajando_con_act=MANDO.trabajar_con_recibida;
          //reset_acciones();//Se Reinicia para Evitar Activaciones accidentales // NOTA:Comentar para DEBUG
          MANDO.trabajar_con_recibida=0;
        }else{
          if(MANDO.enable_enviado_peticion_completo){// Respuesta a las peticiones del MANDO
                MANDO.enable_enviado_peticion_completo=0;
                switch (trabajando_con_act) {
                  case 2:
                    Respuestas_peticiones_para_NCM();
                  break;
                  case 3:
                    Respuestas_peticiones_para_NCL();
                  break;
                  case 4:
                    Respuestas_peticiones_para_RIEL();
                  break;
                  case 5:
                    Respuestas_peticiones_para_ROBOT();
                  break;         
                  default:
                    Respuestas_peticiones_para_CP();
                  break;
                }
                MANDO.peticion_recibida=0;
          }else{
                Actualizar_comunicacion_MANDO();
          }
        }
    }

    void Respuestas_peticiones_para_CP(){
                switch (MANDO.peticion_recibida) {
                  case 1: //Estado_actual
                    enviar_a_MANDO("E"+String(estado_CP_act, DEC));
                  break;
                  case 2: //Modo_actual
                    enviar_a_MANDO("M"+String(modo_CP_act, DEC));
                  break;
                  case 3: //Alertas
                    enviar_a_MANDO("R"+String(error_CP_act, DEC));
                  break;
                  case 4: //Errores
                    enviar_a_MANDO("R"+String(error_CP_act, DEC));
                  break;
//                  case 5: //Estado_Electrovalvulas
//                    enviar_a_MANDO("C"+String(estado_cables_CP_act, DEC));
//                  break;
//                  case 6: //Cables desconectados
//                    enviar_a_MANDO("C"+String(estado_cables_CP_act, DEC));
//                  break;            
                  default:
                    enviar_a_MANDO("C"+String(estado_cables_CP_act, DEC));
                    Serial.print("Actualizar_comunicacion_CP/enviado_peticion: ");
                    Serial.println(MANDO.peticion_recibida);
                  break;
                }
          
    }
    void Respuestas_peticiones_para_NCM(){
                switch (MANDO.peticion_recibida) {
                  case 1: //Estado_actual
                    enviar_a_MANDO("E"+String(estado_NCM_act, DEC));
                  break;
                  case 2: //Modo_actual
                    enviar_a_MANDO("M"+String(modo_NCM_act, DEC));
                  break;
                  case 3: //Alertas
                    enviar_a_MANDO("R"+String(error_NCM_act, DEC));
                  break;
                  case 4: //Errores
                    enviar_a_MANDO("R"+String(error_NCM_act, DEC));
                  break;
                  case 5: //Estado_Electrovalvulas
                    enviar_a_MANDO("V"+String(valvulas_NCM_act, DEC));
                  break;
                  case 6: //Cables desconectados
                    enviar_a_MANDO("C"+String(estado_cables_CP_act, DEC));
                  break;            
                  default:
                    Serial.print("Actualizar_comunicacion_NCM/enviado_peticion: ");
                    Serial.println(MANDO.peticion_recibida);
                  break;
                }
    }
    void Respuestas_peticiones_para_NCL(){
                switch (MANDO.peticion_recibida) {
                  case 1: //Estado_actual
                    enviar_a_MANDO("E"+String(estado_NCL_act, DEC));
                  break;
                  case 2: //Modo_actual
                    enviar_a_MANDO("M"+String(modo_NCL_act, DEC));
                  break;
                  case 3: //Alertas
                    enviar_a_MANDO("R"+String(error_NCL_act, DEC));
                  break;
                  case 4: //Errores
                    enviar_a_MANDO("R"+String(error_NCL_act, DEC));
                  break;
                  case 5: //Estado_Electrovalvulas
                    enviar_a_MANDO("V"+String(valvulas_NCL_act, DEC));
                  break;
                  case 6: //Cables desconectados
                    enviar_a_MANDO("C"+String(estado_cables_CP_act, DEC));
                  break;            
                  default:
                    Serial.print("Actualizar_comunicacion_NCL/enviado_peticion: ");
                    Serial.println(MANDO.peticion_recibida);
                  break;
                }
    }
    void Respuestas_peticiones_para_RIEL(){
                switch (MANDO.peticion_recibida) {
                  case 1: //Estado_actual
                    enviar_a_MANDO("E"+String(estado_RIEL_act, DEC));
                  break;
                  case 2: //Modo_actual
                    enviar_a_MANDO("M"+String(modo_CP_act, DEC));
                  break;
                  case 3: //Alertas
                    enviar_a_MANDO("R"+String(error_CP_act, DEC));
                  break;
                  case 4: //Errores
                    enviar_a_MANDO("R"+String(error_CP_act, DEC));
                  break;
//                  case 5: //Estado_Electrovalvulas
//                    enviar_a_MANDO("C"+String(estado_cables_CP_act, DEC));
//                  break;
//                  case 6: //Cables desconectados
//                    enviar_a_MANDO("C"+String(estado_cables_CP_act, DEC));
//                  break;            
                  default:
                    enviar_a_MANDO("C"+String(estado_cables_CP_act, DEC));
                    Serial.print("Actualizar_comunicacion_RIEL/enviado_peticion: ");
                    Serial.println(MANDO.peticion_recibida);
                  break;
                }
    }
    void Respuestas_peticiones_para_ROBOT(){
                switch (MANDO.peticion_recibida) {
                  case 1: //Estado_actual
                    enviar_a_MANDO("E"+String(estado_ROBOT_act, DEC));
                  break;
                  case 2: //Modo_actual
                    enviar_a_MANDO("M"+String(modo_CP_act, DEC));
                  break;
                  case 3: //Alertas
                    enviar_a_MANDO("R"+String(error_CP_act, DEC));
                  break;
                  case 4: //Errores
                    enviar_a_MANDO("R"+String(error_CP_act, DEC));
                  break;
//                  case 5: //Estado_Electrovalvulas
//                    enviar_a_MANDO("C"+String(estado_cables_CP_act, DEC));
//                  break;
//                  case 6: //Cables desconectados
//                    enviar_a_MANDO("C"+String(estado_cables_CP_act, DEC));
//                  break;
                  default:
                    enviar_a_MANDO("C"+String(estado_cables_CP_act, DEC));
                    Serial.print("Actualizar_comunicacion_NCL/enviado_peticion: ");
                    Serial.println(MANDO.peticion_recibida);
                  break;
                }
    }
    
      void Actualizar_comunicacion_MANDO(){
        if(MANDO.enviado_estado_completo){ // Respuesta se guardan los estados del MANDO
          MANDO.enviado_estado_completo=0;
          estado_MANDO_act=MANDO.estado_recibida;
          MANDO.estado_recibida=0;
        }
        
        if(MANDO.enviado_error_completo){ // Respuesta se guardan los errores del MANDO
              MANDO.enviado_error_completo=0;
              switch (MANDO.error_recibida) {
                case 0: // no error
                  error_MANDO_act=0;
                break;
                case 1: // Joystick_error
                  error_MANDO_act=1;
                break;
                default:
                      Serial.print("Actualizar_comunicacion_CP/enviado_error: ");
                      Serial.println(MANDO.error_recibida);
                break;
              }
              MANDO.error_recibida=0;
        }

        if(MANDO.enviado_accion_completo){
           MANDO.enviado_accion_completo=0;
                switch (MANDO.accion_recibida) {
                  case 1: //Spindler
                    if(MANDO.accion_num_recibida==1){
                      if(trabajando_con_act==2) enviar_a_NCM(F("A11"));
                      if(trabajando_con_act==3) enviar_a_NCL(F("A11"));
                    }else{
                      if(trabajando_con_act==2) enviar_a_NCM(F("A10"));
                      if(trabajando_con_act==3) enviar_a_NCL(F("A10"));
                    }
                  break;
                  case 2: //Cooler
                    if(MANDO.accion_num_recibida==1){
                      if(trabajando_con_act==2) enviar_a_NCM(F("A21"));
                      if(trabajando_con_act==3) enviar_a_NCL(F("A21"));
                    }else{
                      if(trabajando_con_act==2) enviar_a_NCM(F("A20"));
                      if(trabajando_con_act==3) enviar_a_NCL(F("A20"));
                    }
                  break;
                  case 3: //Puerta  
                    if(MANDO.accion_num_recibida==1){
                      if(trabajando_con_act==2) enviar_a_NCM(F("A31"));
                      if(trabajando_con_act==3) enviar_a_NCL(F("A31"));
                    }else{
                      if(trabajando_con_act==2) enviar_a_NCM(F("A30"));
                      if(trabajando_con_act==3) enviar_a_NCL(F("A30"));
                    }
                  break;
                  case 4: //Piston de Agarre
                    if(MANDO.accion_num_recibida==1){
                      if(trabajando_con_act==2) enviar_a_NCM(F("A41"));
                      if(trabajando_con_act==3) enviar_a_NCL(F("A41"));
                    }else{
                      if(trabajando_con_act==2) enviar_a_NCM(F("A40"));
                      if(trabajando_con_act==3) enviar_a_NCL(F("A40"));
                    }
//                    if(trabajando_con_act==2 || trabajando_con_act==3){ // 2 y 3 son NCM y NCL
//                      if(MANDO.accion_num_recibida==1){
//                        bin_recibido_act_valvulas=0b00000001; //operacion or con variable valvulas
//                      }else{
//                        bin_recibido_act_valvulas=0b11111110; //operacion and con variable valvulas
//                      }
//                      enable_activacion_valvulas=1;
//                    }else{
//                      Serial.println("Error_trabajar_con_NCM o NCL");
//                    }
                  break;
                  case 5: //Cambiar Modo Manual/Automatico
                    if(MANDO.accion_num_recibida==1){
                        cambiar_a_modo=1;
                        enable_cambiar_modo=1;
                    }else{
                        cambiar_a_modo=0;
                        enable_cambiar_modo=1;
                    }
                  break;
                  //////////////////////////////////
                  case 10: //Robot ir a posicion # //
                    if(trabajando_con_act==5){ 
                        posicion_a_mover_robot_manual=MANDO.accion_num_recibida;
                        enable_robot_movimiento_a_posicion_manualmente=1;
                    }else{
                        Serial.println("Error_trabajar_con_ROBOT");
                    }
                  break;
                  case 11: //Riel Modo Manual o automatico
                    if(trabajando_con_act==4){
                        if(MANDO.accion_num_recibida==1){
                          enable_riel_movimiento_manual=1; //HABILITA EL MOVIMIENTO CON EL JOYSTICK
                        }else{
                          enable_riel_movimiento_manual=0; //HABILITA EL MOVIMIENTO CON EL JOYSTICK
                        }
                        enable_riel_movimiento_a_posicion_manualmente=0; //HABILITA EL MOVIMIENTO SEMI-AUTOMATICO A UNA POSICION # CON ENTER
                    }else{
                        Serial.println("Error_trabajar_con_RIEL");
                    }
                  break;
                  case 12: //Riel ir a posicion #
                    if(trabajando_con_act==4){
                        posicion_a_mover_riel_manual=MANDO.accion_num_recibida; //LA POSICION # A MOVER CON EL ENTER
                        enable_riel_movimiento_a_posicion_manualmente=1; //HABILITA EL MOVIMIENTO SEMI-AUTOMATICO A UNA POSICION # CON ENTER
                        enable_riel_movimiento_manual=1;  //HABILITA EL MOVIMIENTO CON EL JOYSTICK
                    }else{
                        Serial.println("Error_trabajar_con_RIEL");
                    }
                  break;
                  case 13: // Mover manualmente eje # de NCM
                    if(trabajando_con_act==2){ // 2 son NCM
                      if(MANDO.accion_num_recibida<4){
                          enable_mover_ejes_manualmente=1;
                          ejes_para_mover_manualmente=MANDO.accion_num_recibida;
                      }else{
                        Serial.println("Eje no Existe NCM");
                      }
                    }else{
                      Serial.println("Error_trabajar_con_NCM");
                    }
                  break;
                  case 14: // Mover manualmente eje # de NCL
                    if(trabajando_con_act==3){ // 3 son NCL
                      if(MANDO.accion_num_recibida<3){
                          enable_mover_ejes_manualmente=1;
                          ejes_para_mover_manualmente=MANDO.accion_num_recibida;
                      }else{
                        Serial.println("Eje no Existe NCL");
                      }
                    }else{
                      Serial.println("Error_trabajar_con_NCL");
                    }
                  break;            
                  default:
                      Serial.print("Actualizar_comunicacion_CP/accion: ");
                      Serial.println(MANDO.accion_recibida);
                  break;
                }
          MANDO.accion_recibida=0;
          MANDO.accion_num_recibida=0;
        }
        if(MANDO.enviado_velocidad_completo){
          MANDO.enviado_velocidad_completo=0;
            velocidad_MANDO_act=MANDO.velocidad_recibida;
            enable_cambiar_velocidad=1;
          MANDO.velocidad_recibida=0;
        }
      }
////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////
////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////
////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////////RESPUESTA_A _COMANDOS_MANDO////

////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////
////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////
////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////
void Recepcion_Dato_NCL(){
  char c1 = Serial1.read();
  NCL.rev=1;
  if(NCL.enviando_comando_tipo==0){
    Buscar_comandos_NCL(c1);
  }
  if(NCL.rev) grafcet_NCL(c1);
}
void Buscar_comandos_NCL(char c){ // Todos los comandos que puede enviar el MANDO
  if(NCL.enviando_comando_tipo==0){
    NCL.rev=0;
    switch (c) {
      case 'E': // NCL.estado_recibida
        NCL.enviando_comando_tipo=3;
        NCL.EE0=0;
        NCL.EE1=1;
      break;
      case 'R': //error
        NCL.enviando_comando_tipo=4;
        NCL.ER0=0;
        NCL.ER1=1;
      break;
      case 'S': //velocidad
        NCL.enviando_comando_tipo=6;
        NCL.ES0=0;
        NCL.ES1=1;
      break;
      case 'V': //valvulas
        NCL.enviando_comando_tipo=7;
        NCL.EV1=1;
      break;
      case 'J': //Eje trabajo manual
        NCL.enviando_comando_tipo=10;
        NCL.EJ0=0;
        NCL.EJ1=1;
      break;      
      default:
        NCL.rev=1;
        Serial.print("Error No se encuentra NCL");
        Serial.println(c);
      break;
    }
  }
}
//////////////////////////////////////////////////////////////////////////////GRACET COMUNICACION//////////////////////////////////////////////////////////////
void grafcet_NCL(char c){
    switch (NCL.enviando_comando_tipo) {
    case 3:
      grafcet_estados_NCL(c);   //guarda el estado del MANDO
    break;
    case 4:
      grafcet_errores_NCL(c);   //guarda el estado del MANDO
    break;
    case 6:
      grafcet_velocidad_NCL(c); // solo funciona cuando se trabaja con NCL y NCM
    break;
    case 7:
      grafcet_valvulas_NCL(c);
    break;
    case 10:
      grafcet_eje_NCL(c);
    break;
    default:
    break;
  }
}

void grafcet_estados_NCL(char c){
    if(NCL.EE1){
      NCL.EE1=0;
      NCL.EE2=1;
      NCL.rev=0;
      switch (c) {
        case '0': //Preparado
          NCL.estado_recibida=0;
        break;
        case '1'://Ocupado
          NCL.estado_recibida=1;
        break;
        case '2'://Con errores o Alertas
          NCL.estado_recibida=2;
        break;
        case '3'://deshabilitado
          NCL.estado_recibida=3;
        break;
        case '4'://Trabajando
          NCL.estado_recibida=4;
        break;
        case '5'://Paro de Emergencia
          NCL.estado_recibida=5;
        break;        
        default:
          NCL.rev=1;
          NCL.EE0=1;
          NCL.EE2=0;
          NCL.enviando_comando_tipo=0;
          Serial.print("Error_E: ");
          Serial.println(c);
        break;
      }
    }
    if(NCL.EE2 && NCL.rev){
      NCL.EE0=1;
      NCL.EE2=0;
      NCL.enviando_comando_tipo=0;
      if(c==10 || c==13){
        NCL.enviado_estado_completo=1;
        NCL.recibido_nuevo_comando=1;
        Serial.print("NCL.estado_recibida: ");
        Serial.println(NCL.estado_recibida);
        NCL.rev=0;
      }
    }
}
void grafcet_errores_NCL(char c){
  if(NCL.ER1){
    NCL.ER1=0;
    NCL.ER2=1;
    NCL.rev=0;
    if(isdigit(c)){
      NCL.error_recibida+=String(c).toInt();
    }else{
      NCL.rev=1;
      NCL.ER0=1;
      NCL.ER2=0;
      NCL.enviando_comando_tipo=0;
      Serial.print("Error_E: ");
      Serial.println(c);
    }
  }
  if(NCL.ER2 && NCL.rev){
    NCL.ER0=1;
    NCL.ER2=0;
    NCL.enviando_comando_tipo=0;
    if(c==10 || c==13){
      NCL.enviado_error_completo=1;
      NCL.recibido_nuevo_comando=1;
      Serial.print("error: ");
      Serial.println(NCL.error_recibida);
      NCL.rev=0;
    }
  }
}

void grafcet_velocidad_NCL(char c){
  if(NCL.ES1){
    NCL.ES1=0;
    NCL.ES2=1;
    NCL.rev=0;
    if(isdigit(c)){
      NCL.velocidad_acu_recibida+=c;
    }else{
      NCL.rev=1;
      NCL.ES0=1;
      NCL.ES2=0;
      NCL.velocidad_acu_recibida="";
      NCL.enviando_comando_tipo=0;
      Serial.print("Error_Speed1: ");
      Serial.println(c);
    }
  }
  if(NCL.ES2 && NCL.rev){
    NCL.ES2=0;
    NCL.ES3=1;
    NCL.rev=0;
    if(isdigit(c)){
      NCL.velocidad_acu_recibida+=c;
      NCL.velocidad_recibida=String(NCL.velocidad_acu_recibida).toInt();
      NCL.velocidad_acu_recibida="";
    }else{
      NCL.rev=1;
      NCL.ES0=1;
      NCL.ES3=0;
      NCL.velocidad_acu_recibida="";
      NCL.enviando_comando_tipo=0;
      Serial.print("Error_Speed2: ");
      Serial.println(c);
    }
  }
  if(NCL.ES3 && NCL.rev){
    NCL.ES3=0;
    NCL.ES0=1;
    NCL.enviando_comando_tipo=0;
    if(c==10 || c==13){
      NCL.enviado_velocidad_completo=1;
      NCL.recibido_nuevo_comando=1;
      Serial.print("NCL.velocidad_recibida");
      Serial.println(NCL.velocidad_recibida);
      NCL.rev=0;
    }else{
      Serial.print("falto_enter: ");
      Serial.println(c);
    }
  }
}
void grafcet_valvulas_NCL(char c){
  if(NCL.EV1){
    NCL.EV1=0;
    NCL.EV2=1;
    NCL.rev=0;
    if(isdigit(c)){
      NCL.valvulas_recibida=String(c).toInt()*10;
    }else{
      NCL.rev=1;
      NCL.EV2=0;
      NCL.valvulas_recibida=0;
      NCL.enviando_comando_tipo=0;
      Serial.print(F("Error_valvulas_#1: "));
      Serial.println(c);
    }
  }
  if(NCL.EV2 && NCL.rev){
    NCL.EV2=0;
    NCL.EV3=1;
    NCL.rev=0;
    if(isdigit(c)){
      NCL.valvulas_recibida+=String(c).toInt();
    }else{
      NCL.rev=1;
      NCL.EV3=0;
      NCL.valvulas_recibida=0;
      NCL.enviando_comando_tipo=0;
      Serial.print(F("Error_valvulas_#1: "));
      Serial.println(c);
    }
  }
  if(NCL.EV3 && NCL.rev){
    NCL.EV3=0;
    NCL.enviando_comando_tipo=0;
    if(c==10 || c==13){
      if(NCL.valvulas_recibida<16){
          NCL.enviado_valvulas_completo=1;
          NCL.recibido_nuevo_comando=1;
          Serial.print(F("NCL.valvulas_recibida"));
          Serial.println(NCL.valvulas_recibida);
          NCL.rev=0;
      }else{
        Serial.print(F("Valvu_fuera_rango: "));
        Serial.println(NCL.valvulas_recibida);
      }
    }else{
      Serial.print(F("falto_enter_V: "));
      Serial.println(c);
    }
  }
}
void grafcet_eje_NCL(char c){
  if(NCL.EJ1){
    NCL.EJ1=0;
    NCL.EJ2=1;
    NCL.rev=0;
    if(isdigit(c)){
      NCL.eje_recibida+=String(c).toInt();
    }else{
      NCL.rev=1;
      NCL.EJ0=1;
      NCL.EJ2=0;
      NCL.enviando_comando_tipo=0;
      Serial.print("Error_valvulas: ");
      Serial.println(c);
    }
  }
  if(NCL.EJ2 && NCL.rev){
    NCL.EJ2=0;
    NCL.EJ0=1;
    NCL.enviando_comando_tipo=0;
    if(c==10 || c==13){
      NCL.enviado_eje_completo=1;
      NCL.recibido_nuevo_comando=1;
      Serial.print("NCL.valvulas_recibida ");
      Serial.println(NCL.valvulas_recibida);
      NCL.rev=0;
    }else{
      Serial.print("falto_enter: ");
      Serial.println(c);
    }
  }
}
////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////
////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////
////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////
      void Respuesta_a_comandos_NCL(){
        if(NCL.enviado_estado_completo){ // Se guarda el estado de_NCL
          NCL.enviado_estado_completo=0;
          estado_NCL_act=NCL.estado_recibida;
          NCL.estado_recibida=0;
        }
        if(NCL.enviado_error_completo){ // Se guarda el error de_NCL
          NCL.enviado_error_completo=0;
          error_NCL_act=NCL.error_recibida;
          NCL.error_recibida=0;
        }
        if(NCL.enviado_velocidad_completo){ // Se guarda la velocidad de_NCL
          NCL.enviado_velocidad_completo=0;
          velocidad_NCL_act=NCL.velocidad_recibida;
          NCL.velocidad_recibida=0;
        }
        if(NCL.enviado_valvulas_completo){ // Se guarda las valvulas de_NCL
          NCL.enviado_valvulas_completo=0;
          valvulas_NCL_act=NCL.valvulas_recibida;
          NCL.valvulas_recibida=0;
        }
        if(NCL.enviado_eje_completo){ // Se guarda el eje de_NCL
          NCL.enviado_eje_completo=0;
          if(NCL.eje_recibida<3){ // solo se permite Eje,0,1y2
            eje_NCL_act=NCL.eje_recibida;
          }else{
            Serial.print("NCL_Error_Eje_no_permitido-");
            Serial.println(NCL.eje_recibida);
          }
          NCL.eje_recibida=0;
        }        

      }
////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////
////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////
////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////////RESPUESTA_A _COMANDOS_NCL////
////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////
////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////
////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////////COMUNICACION_NCL////

////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////
////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////
////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////
void Recepcion_Dato_NCM(){
  char c1 = Serial2.read();
  NCM.rev=1;
  if(NCM.enviando_comando_tipo==0){
    Buscar_comandos_NCM(c1);
  }
  if(NCM.rev) grafcet_NCM(c1);
}
void Buscar_comandos_NCM(char c){ // Todos los comandos que puede enviar el MANDO
  if(NCM.enviando_comando_tipo==0){
    NCM.rev=0;
    switch (c) {
      case 'E': // NCM.estado_recibida
        NCM.enviando_comando_tipo=3;
        NCM.EE0=0;
        NCM.EE1=1;
      break;
      case 'R': //error
        NCM.enviando_comando_tipo=4;
        NCM.ER0=0;
        NCM.ER1=1;
      break;
      case 'S': //velocidad
        NCM.enviando_comando_tipo=6;
        NCM.ES0=0;
        NCM.ES1=1;
      break;
      case 'V': //valvulas
        NCM.enviando_comando_tipo=7;
        NCM.EV1=1;
      break;
      case 'J': //Eje trabajo manual
        NCM.enviando_comando_tipo=10;
        NCM.EJ0=0;
        NCM.EJ1=1;
      break;      
      default:
        NCM.rev=1;
        Serial.print("Error No se encuentra NCM");
        Serial.println(c);
      break;
    }
  }
}
//////////////////////////////////////////////////////////////////////////////GRACET COMUNICACION//////////////////////////////////////////////////////////////
void grafcet_NCM(char c){
    switch (NCM.enviando_comando_tipo) {
    case 3:
      grafcet_estados_NCM(c);   //guarda el estado del MANDO
    break;
    case 4:
      grafcet_errores_NCM(c);   //guarda el estado del MANDO
    break;
    case 6:
      grafcet_velocidad_NCM(c); // solo funciona cuando se trabaja con NCL y NCM
    break;
    case 7:
      grafcet_valvulas_NCM(c);
    break;
    case 10:
      grafcet_eje_NCM(c);
    break;
    default:
    break;
  }
}

void grafcet_estados_NCM(char c){
    if(NCM.EE1){
      NCM.EE1=0;
      NCM.EE2=1;
      NCM.rev=0;
      switch (c) {
        case '0': //Preparado
          NCM.estado_recibida=0;
        break;
        case '1'://Ocupado
          NCM.estado_recibida=1;
        break;
        case '2'://Con errores o Alertas
          NCM.estado_recibida=2;
        break;
        case '3'://deshabilitado
          NCM.estado_recibida=3;
        break;
        case '4'://Trabajando
          NCM.estado_recibida=4;
        break;
        case '5'://Paro de Emergencia
          NCM.estado_recibida=5;
        break;        
        default:
          NCM.rev=1;
          NCM.EE0=1;
          NCM.EE2=0;
          NCM.enviando_comando_tipo=0;
          Serial.print("Error_E: ");
          Serial.println(c);
        break;
      }
    }
    if(NCM.EE2 && NCM.rev){
      NCM.EE0=1;
      NCM.EE2=0;
      NCM.enviando_comando_tipo=0;
      if(c==10 || c==13){
        NCM.enviado_estado_completo=1;
        NCM.recibido_nuevo_comando=1;
        Serial.print("NCM.estado_recibida: ");
        Serial.println(NCM.estado_recibida);
        NCM.rev=0;
      }
    }
}
void grafcet_errores_NCM(char c){
  if(NCM.ER1){
    NCM.ER1=0;
    NCM.ER2=1;
    NCM.rev=0;
    if(isdigit(c)){
      NCM.error_recibida+=String(c).toInt();
    }else{
      NCM.rev=1;
      NCM.ER0=1;
      NCM.ER2=0;
      NCM.enviando_comando_tipo=0;
      Serial.print("Error_E: ");
      Serial.println(c);
    }
  }
  if(NCM.ER2 && NCM.rev){
    NCM.ER0=1;
    NCM.ER2=0;
    NCM.enviando_comando_tipo=0;
    if(c==10 || c==13){
      NCM.enviado_error_completo=1;
      NCM.recibido_nuevo_comando=1;
      Serial.print("error: ");
      Serial.println(NCM.error_recibida);
      NCM.rev=0;
    }
  }
}

void grafcet_velocidad_NCM(char c){
  if(NCM.ES1){
    NCM.ES1=0;
    NCM.ES2=1;
    NCM.rev=0;
    if(isdigit(c)){
      NCM.velocidad_acu_recibida+=c;
    }else{
      NCM.rev=1;
      NCM.ES0=1;
      NCM.ES2=0;
      NCM.velocidad_acu_recibida="";
      NCM.enviando_comando_tipo=0;
      Serial.print("Error_Speed1: ");
      Serial.println(c);
    }
  }
  if(NCM.ES2 && NCM.rev){
    NCM.ES2=0;
    NCM.ES3=1;
    NCM.rev=0;
    if(isdigit(c)){
      NCM.velocidad_acu_recibida+=c;
      NCM.velocidad_recibida=String(NCM.velocidad_acu_recibida).toInt();
      NCM.velocidad_acu_recibida="";
    }else{
      NCM.rev=1;
      NCM.ES0=1;
      NCM.ES3=0;
      NCM.velocidad_acu_recibida="";
      NCM.enviando_comando_tipo=0;
      Serial.print("Error_Speed2: ");
      Serial.println(c);
    }
  }
  if(NCM.ES3 && NCM.rev){
    NCM.ES3=0;
    NCM.ES0=1;
    NCM.enviando_comando_tipo=0;
    if(c==10 || c==13){
      NCM.enviado_velocidad_completo=1;
      NCM.recibido_nuevo_comando=1;
      Serial.print("NCM.velocidad_recibida");
      Serial.println(NCM.velocidad_recibida);
      NCM.rev=0;
    }else{
      Serial.print("falto_enter: ");
      Serial.println(c);
    }
  }
}
void grafcet_valvulas_NCM(char c){
  if(NCM.EV1){
    NCM.EV1=0;
    NCM.EV2=1;
    NCM.rev=0;
    if(isdigit(c)){
      NCM.valvulas_recibida+=String(c).toInt();
    }else{
      NCM.rev=1;
      NCM.EV2=0;
      NCM.valvulas_recibida=0;
      NCM.enviando_comando_tipo=0;
      Serial.print("Error_valvulas_#1: ");
      Serial.println(c);
    }
  }
  if(NCM.EV2 && NCM.rev){
    NCM.EV2=0;
    NCM.EV3=1;
    NCM.rev=0;
    if(isdigit(c)){
      NCM.valvulas_recibida+=String(c).toInt();
    }else{
      NCM.rev=1;
      NCM.EV3=0;
      NCM.valvulas_recibida=0;
      NCM.enviando_comando_tipo=0;
      Serial.print(F("Error_valvulas_#2: "));
      Serial.println(c);
    }
  }
  if(NCM.EV3 && NCM.rev){
    NCM.EV3=0;
    NCM.enviando_comando_tipo=0;
    if(c==10 || c==13){
      if(NCM.valvulas_recibida<16){
          NCM.enviado_valvulas_completo=1;
          NCM.recibido_nuevo_comando=1;
          Serial.print(F("NCM.valvulas_recibida"));
          Serial.println(NCM.valvulas_recibida);
          NCM.rev=0;
      }else{
        Serial.print(F("Valvu_fuera_rango: "));
        Serial.println(NCM.valvulas_recibida);
      }
    }else{
      Serial.print(F("falto_enter_V: "));
      Serial.println(c);
    }
  }
}
void grafcet_eje_NCM(char c){
  if(NCM.EJ1){
    NCM.EJ1=0;
    NCM.EJ2=1;
    NCM.rev=0;
    if(isdigit(c)){
      NCM.eje_recibida+=String(c).toInt();
    }else{
      NCM.rev=1;
      NCM.EJ0=1;
      NCM.EJ2=0;
      NCM.enviando_comando_tipo=0;
      Serial.print("Error_valvulas: ");
      Serial.println(c);
    }
  }
  if(NCM.EJ2 && NCM.rev){
    NCM.EJ2=0;
    NCM.EJ0=1;
    NCM.enviando_comando_tipo=0;
    if(c==10 || c==13){
      NCM.enviado_eje_completo=1;
      NCM.recibido_nuevo_comando=1;
      Serial.print("NCM.valvulas_recibida ");
      Serial.println(NCM.valvulas_recibida);
      NCM.rev=0;
    }else{
      Serial.print("falto_enter: ");
      Serial.println(c);
    }
  }
}
////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////
////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////
////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////
      void Respuesta_a_comandos_NCM(){
        if(NCM.enviado_estado_completo){ // Se guarda el estado de_NCM
          NCM.enviado_estado_completo=0;
          estado_NCM_act=NCM.estado_recibida;
          NCM.estado_recibida=0;
        }
        if(NCM.enviado_error_completo){ // Se guarda el error de_NCM
          NCM.enviado_error_completo=0;
          error_NCM_act=NCM.error_recibida;
          NCM.error_recibida=0;
        }
        if(NCM.enviado_velocidad_completo){ // Se guarda la velocidad de_NCM
          NCM.enviado_velocidad_completo=0;
          velocidad_NCM_act=NCM.velocidad_recibida;
          NCM.velocidad_recibida=0;
        }
        if(NCM.enviado_valvulas_completo){ // Se guarda las valvulas de_NCM
          NCM.enviado_valvulas_completo=0;
          valvulas_NCM_act=NCM.valvulas_recibida;
          NCM.valvulas_recibida=0;
        }
        if(NCM.enviado_eje_completo){ // Se guarda el eje de_NCM
          NCM.enviado_eje_completo=0;
          if(NCM.eje_recibida<4){ // solo se permite Eje,0,1y2
            eje_NCM_act=NCM.eje_recibida;
          }else{
            Serial.print("NCM_Error_Eje-");
            Serial.println(NCM.eje_recibida);
          }
          NCM.eje_recibida=0;
        }        
      }

////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////
////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////
////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////////RESPUESTA_A _COMANDOS_NCM////
////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////
////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////
////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////////COMUNICACION_NCM////


