//    profundidad_en_menu=1; //trabajar con                       /posicion_en_el_menu el max5
//    profundidad_en_menu=2; //informacion / acciones             /posicion_en_el_menu el max2
//    profundidad_en_menu=3; //informacion                        /posicion_en_el_menu el max5
//    profundidad_en_menu=4; // acciones_CP                       /posicion_en_el_menu el max1
//    profundidad_en_menu=5; // acciones_NCM                      /posicion_en_el_menu el max3
//    profundidad_en_menu=6; // acciones_NCL                      /posicion_en_el_menu el max3
//    profundidad_en_menu=7; // acciones_riel                     /posicion_en_el_menu el max1
//    profundidad_en_menu=8; // acciones_robot                    /posicion_en_el_menu el max1

#define lcd_print(cadena){ lcd.print(cadena); }
#define enviar_a_CP(comando){Serial.println(comando);}
#define SERIAL_PRINT(comando){Serial.print(comando);}
#define SERIAL_PRINTLN(comando){Serial.println(comando);}

int cosa=1;
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#define  PIN_DIR 12
#define  PIN_PUL 13
#define  PIN_AUX1 A1
#define  PIN_AUX2 A2
#define ascii_flechaI 127 //<
#define ascii_flechaD 126 //>
LiquidCrystal_I2C lcd(0x27,16,2); 
const byte Filas = 4;          //Cuatro filas
const byte Cols = 4;           //Cuatro columnas
byte Pins_Filas[] = {7, 6, 5, 4};      //Pines Arduino para las filas
byte Pins_Cols[] = {11, 10, 9, 8};     // Pines Arduinopara las columnas
char Teclas [ Filas ][ Cols ] =
    {
        {'1','2','3','A'},
        {'4','5','6','B'},
        {'7','8','9','C'},
        {'*','0','#','D'}
    };
Keypad Teclado1 = Keypad(makeKeymap(Teclas), Pins_Filas, Pins_Cols, Filas, Cols);

struct MANDO_VAR {
bool EP1; bool EP2;  bool ET1; bool ET2; bool EE1; bool EE2; bool EM1; bool ER1;
bool ER2;  bool EV1; bool EV2; bool EV3 ;bool EC1; bool EC2; bool EC3; bool EM2;
bool rev; uint8_t enviando_comando_tipo;
bool enable_enviado_peticion_completo; uint8_t peticion_recibida;
bool enviado_trabajo_completo; uint8_t trabajar_con_recibida;
bool enviado_estado_completo; uint8_t estado_recibida; 
bool enviado_error_completo; uint8_t error_recibida;
bool enviado_valvulas_completo; uint8_t valvulas_recibida;
bool enviado_cables_completo; uint8_t cables_recibida;
bool enviado_modo_completo; bool modo_recibida;
bool recibido_nuevo_comando;
};

MANDO_VAR MANDO={0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,  0,0,0,0};

String atras_joy="Atras[*]  <JOY>";

int velocidad=0;
unsigned long t_ultima_envio_a_CP;
unsigned long t_ultima_actualizacion_lcd;
bool pul_act=0;
bool dir_act=0;
bool enable_rutina_inicio=0;
char pulsacion;
bool E0=1; bool E1=0; bool E2=0; bool pulso=0;
bool EW0=1; bool EW1=0; bool EW2=0; bool enter=0;
bool mov_der=0; bool mov_izq=0;

uint8_t trabajando_con_act=0;
uint8_t profundidad_en_menu=0;
uint8_t posicion_en_el_menu=0;

bool enable_enviar_accion_CP=0;
bool enable_enviar_accion_NCM=0;
bool enable_enviar_accion_NCL=0;
bool enable_enviar_accion_RIEL=0;
bool enable_enviar_accion_ROBOT=0;
bool enable_actualizar_lcd=0;
bool enable_PUL_DIR_OUTPUT=0;

uint8_t num_CP_selecionado=0;
uint8_t num_NCM_eje_selecionado=0;
uint8_t num_NCM_eje_act=0;
uint8_t num_NCM_valvula_selecionado=0;
uint8_t num_NCL_eje_selecionado=0;
uint8_t num_NCL_eje_act=0;
uint8_t num_NCL_valvula_selecionado=0;
uint8_t num_ROBOT_selecionado=0;
uint8_t num_RIEL_selecionado=0;

bool EPP0=1; bool EPP1=0; bool EPP2=0; bool EPP3=0; bool EPP4=0; bool EPP5=0;
bool enable_enviar_peticiones_segun_trajando_con=0;

uint8_t estado_recibido_CP=0; //variable de informacion
uint8_t error_recibido_CP=0;  //variable de informacion
bool modo_recibido_CP=0;      //variable de informacion
uint8_t informacion_Cables=0;      //variable de informacion
uint8_t informacion_valvulas_NCM=0;      //variable de informacion
uint8_t informacion_valvulas_NCL=0;      //variable de informacion
//uint8_t Debug_Comunicacion;      //variable de informacion

bool enable_apagar_al_desactivar_modo_manual_NCM=0;
bool enable_apagar_al_desactivar_modo_manual_NCL=0;
bool enable_apagar_al_desactivar_modo_manual_RIEL=0;

void setup() {
  pinMode(A0, INPUT);        //Potenciometro
  pinMode(A1, OUTPUT);       //AUX1
  pinMode(A2, OUTPUT);       //AUX2
  pinMode(A3, INPUT_PULLUP); //SW Joystick
  pinMode(12, OUTPUT);       //DIR
  pinMode(13, OUTPUT);       //PUL
  lcd.init();
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd_print(F("FMS-2101"));
  lcd.setCursor(0, 1);
  lcd_print(F("Bienvenido"));
  delay(900);
  lcd.scrollDisplayRight();
  delay(700);
  lcd.scrollDisplayRight();
  delay(500);
  lcd.scrollDisplayRight();
  delay(2000);
  lcd.clear();
  digitalWrite(PIN_DIR,0);
  digitalWrite(PIN_PUL,0);
  digitalWrite(PIN_AUX1,0);
  digitalWrite(PIN_AUX2,0);
  Serial.begin(9600);
  profundidad_en_menu=1;
  enable_actualizar_lcd=1;
  enviar_a_CP(F("E0"));
  enviar_a_CP(F("R0"));
  enviar_a_CP(F("T0"));
  t_ultima_actualizacion_lcd=millis();
  t_ultima_envio_a_CP=t_ultima_actualizacion_lcd;
}

void loop() {
    if (Serial.available() > 0) {
      Recepcion_Dato_MANDO();
    }
    if(MANDO.recibido_nuevo_comando){
      MANDO.recibido_nuevo_comando=0;
      Respuesta_a_comandos_MANDO();
      enable_actualizar_lcd=1;
    }
    Control_Principal();
    pulsacion = Teclado1.getKey();
    if (pulsacion != 0){
      enable_actualizar_lcd=1;
      //SERIAL_PRINTLN(pulsacion);
      //debug();
    }
    verificar_joystick_enter();
    if(enter || pulsacion=='D'){ //Enter
      validar_enter();
      enable_actualizar_lcd=1;
      //debug();
    }
    if(pulsacion=='*'){ // Tecla * - atras
      validar_atras();
      enable_actualizar_lcd=1;
      //debug();
    }
    verificar_joystick_mov_horizontal();
    if(mov_der || mov_izq || pulsacion=='#'){
      validar_posicion_menu();
      enable_actualizar_lcd=1;
      //debug();
    }
    Actualizar_LCD();
}

//void debug(){
//  SERIAL_PRINT("^"); SERIAL_PRINT(profundidad_en_menu);
//  SERIAL_PRINT(">"); SERIAL_PRINT(posicion_en_el_menu);
//  SERIAL_PRINT("P"); SERIAL_PRINT(enable_PUL_DIR_OUTPUT);
//  SERIAL_PRINT("T"); SERIAL_PRINTLN(trabajando_con_act);
//  SERIAL_PRINT("num_CP_selecionado  "); SERIAL_PRINTLN(num_CP_selecionado);
//  SERIAL_PRINT("num_NCM_valvula_selecionado  "); SERIAL_PRINTLN(num_NCM_valvula_selecionado);
//  SERIAL_PRINT("num_NCM_eje_selecionado  "); SERIAL_PRINTLN(num_NCM_eje_selecionado);
//  SERIAL_PRINT("num_NCL_valvula_selecionado  "); SERIAL_PRINTLN(num_NCL_valvula_selecionado);
//  SERIAL_PRINT("num_NCL_eje_selecionado  "); SERIAL_PRINTLN(num_NCL_eje_selecionado);
//  SERIAL_PRINT("num_RIEL_selecionado  "); SERIAL_PRINTLN(num_RIEL_selecionado);
//  SERIAL_PRINT("num_ROBOT_selecionado  "); SERIAL_PRINTLN(num_ROBOT_selecionado);
//  SERIAL_PRINTLN();
//}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Control_Principal(){
    if(enable_enviar_peticiones_segun_trajando_con){
      enviar_peticiones_segun_trabajando();
    }
    if(posicion_en_el_menu==0 && (profundidad_en_menu==5 ||  profundidad_en_menu==6) ){
        velocidad=get_pot_val(A0);
        lcd.setCursor(13, 0);
        if(velocidad<10){
          lcd.print(" ");
          lcd.setCursor(14, 0);
          lcd.print(velocidad);
        }else{
          lcd.print(velocidad);
        }
    }
    if(enable_enviar_accion_CP && num_CP_selecionado!=0){
        enable_enviar_accion_CP=0;
        if(num_CP_selecionado==1){
          enviar_a_CP(F("AC0"));
        }
        if(num_CP_selecionado==2){
          enviar_a_CP(F("AC1"));
        }
    }
    if(enable_enviar_accion_NCM){
        enable_enviar_accion_NCM=0;
    
        if(posicion_en_el_menu==0){
            if(velocidad<10){
              enviar_a_CP("S0"+String(velocidad, DEC));
            }else{
              enviar_a_CP("S"+String(velocidad, DEC));
            }
        }
        if(posicion_en_el_menu==1 && num_NCM_valvula_selecionado!=0){
              if(num_NCM_valvula_selecionado==1){ //"CERRAR Puerta?"
                enviar_a_CP(F("A40"));
              }
              if(num_NCM_valvula_selecionado==2){//"OFF - Agarre?"
                enviar_a_CP(F("A30"));
              }
              if(num_NCM_valvula_selecionado==3){//"OFF - Cooler?"
                enviar_a_CP(F("A20"));
              }
              if(num_NCM_valvula_selecionado==4){//"OFF - Spindle?"
                enviar_a_CP(F("A10"));
              }
              if(num_NCM_valvula_selecionado==5){//"ABRIR  Puerta?"
                enviar_a_CP(F("A41"));
              }
              if(num_NCM_valvula_selecionado==6){//ON  - Agarre?
                enviar_a_CP(F("A31"));
              }
              if(num_NCM_valvula_selecionado==7){//ON  - Cooler?
                enviar_a_CP(F("A21"));
              }
              if(num_NCM_valvula_selecionado==8){//ON  - Spindle?
                enviar_a_CP(F("A11"));
              }  
        }
        if(posicion_en_el_menu==2 && num_NCM_eje_selecionado!=0){
            if(num_NCM_eje_selecionado>0){
              enviar_a_CP("AM"+String(num_NCM_eje_selecionado, DEC));
              enable_PUL_DIR_OUTPUT=1;
              enable_apagar_al_desactivar_modo_manual_NCM=1;
              num_NCM_eje_act=num_NCM_eje_selecionado;
            }else{
              enviar_a_CP(F("AM0"));
              enable_PUL_DIR_OUTPUT=0;
            }
        }
    }
    if(enable_enviar_accion_NCL){
        enable_enviar_accion_NCL=0;
        
        if(posicion_en_el_menu==0){
            if(velocidad<10){
              enviar_a_CP("S0"+String(velocidad, DEC));
            }else{
              enviar_a_CP("S"+String(velocidad, DEC));
            }
        }
        if(posicion_en_el_menu==1 && num_NCL_valvula_selecionado!=0){
              if(num_NCL_valvula_selecionado==1){ //"CERRAR Puerta?"
                enviar_a_CP(F("A40"));
              }
              if(num_NCL_valvula_selecionado==2){//"OFF - Agarre?"
                enviar_a_CP(F("A30"));
              }
              if(num_NCL_valvula_selecionado==3){//"OFF - Cooler?"
                enviar_a_CP(F("A20"));
              }
              if(num_NCL_valvula_selecionado==4){//"OFF - Spindle?"
                enviar_a_CP(F("A10"));
              }
              if(num_NCL_valvula_selecionado==5){//"ABRIR  Puerta?"
                enviar_a_CP(F("A41"));
              }
              if(num_NCL_valvula_selecionado==6){//ON  - Agarre?
                enviar_a_CP(F("A31"));
              }
              if(num_NCL_valvula_selecionado==7){//ON  - Cooler?
                enviar_a_CP(F("A21"));
              }
              if(num_NCL_valvula_selecionado==8){//ON  - Spindle?
                enviar_a_CP(F("A11"));
              }  
        }
        if(posicion_en_el_menu==2 && num_NCL_eje_selecionado!=0){
            if(num_NCL_eje_selecionado>0){
              enviar_a_CP("AL"+String(num_NCL_eje_selecionado, DEC));
              enable_PUL_DIR_OUTPUT=1;
              enable_apagar_al_desactivar_modo_manual_NCL=1;
              num_NCL_eje_act=num_NCL_eje_selecionado;
            }else{
              enviar_a_CP(F("AL0"));
              enable_PUL_DIR_OUTPUT=0;
            }
        }
    }
    if(enable_PUL_DIR_OUTPUT && (posicion_en_el_menu==2)){
            if(num_NCM_eje_act!=num_NCM_eje_selecionado && profundidad_en_menu==5){
              num_NCM_eje_act=num_NCM_eje_selecionado;
              enviar_a_CP("AM"+String(num_NCM_eje_selecionado, DEC));
              enable_apagar_al_desactivar_modo_manual_NCM=1;
            }
            if(num_NCL_eje_act!=num_NCL_eje_selecionado && profundidad_en_menu==6){
              num_NCL_eje_act=num_NCL_eje_selecionado;
              enviar_a_CP("AL"+String(num_NCL_eje_selecionado, DEC));
              enable_apagar_al_desactivar_modo_manual_NCL=1;
            }
    }

    if(enable_enviar_accion_RIEL){
        enable_enviar_accion_RIEL=0;
        if(num_RIEL_selecionado==1){
          enviar_a_CP(F("AI0"));
          enable_PUL_DIR_OUTPUT=0;
        }
        if(num_RIEL_selecionado==2){
          enviar_a_CP(F("AI1"));
          enable_PUL_DIR_OUTPUT=1;
          enable_apagar_al_desactivar_modo_manual_RIEL=1;
        }
    }
    if(enable_enviar_accion_ROBOT){ // se envia la posicion del robot
        enable_enviar_accion_ROBOT=0;
        enviar_a_CP("AR"+String(num_ROBOT_selecionado, DEC));
        enable_PUL_DIR_OUTPUT=1;
    }
    if(!enable_PUL_DIR_OUTPUT && enable_apagar_al_desactivar_modo_manual_RIEL){
      //Se envia la desactivacion cuando finaliza su uso
      enable_apagar_al_desactivar_modo_manual_RIEL=0;
      enviar_a_CP(F("AI0"));
    }
    if(!enable_PUL_DIR_OUTPUT && enable_apagar_al_desactivar_modo_manual_NCM){
      //Se envia la desactivacion cuando finaliza su uso
      enable_apagar_al_desactivar_modo_manual_NCM=0;
      enviar_a_CP(F("AM0"));
    }
    if(!enable_PUL_DIR_OUTPUT && enable_apagar_al_desactivar_modo_manual_NCL){
      //Se envia la desactivacion cuando finaliza su uso
      enable_apagar_al_desactivar_modo_manual_NCL=0;
      enviar_a_CP(F("AL0"));
    }        
}

//
void get_joystick_comand(uint8_t pin){
  int analoginput=analogRead(pin);
  if(analoginput>=497 && analoginput<=527){
    pul_act=0;
    dir_act=0;
    //velocidad=0;
  }else{
    if(analoginput>527){
      analoginput=analoginput-527;
      dir_act=1;
    }else{
      analoginput=497-analoginput;
      dir_act=0;
    }
    //velocidad=map(analoginput, 0, 490, 0, 9); //497 a un lado da como maximo 9 y al otro solo 8 para eso cambie
    pul_act=1;
  }
  if(enable_PUL_DIR_OUTPUT){
        if(pul_act){
          digitalWrite(PIN_PUL,1);
        }else{
          digitalWrite(PIN_PUL,0);
        }
        if(dir_act){
          digitalWrite(PIN_DIR,1);
        }else{
          digitalWrite(PIN_DIR,0);
        }
    }else{
        digitalWrite(PIN_PUL,0);
        digitalWrite(PIN_DIR,0);
    }
}

int get_pot_val(uint8_t pin){
  int analoginput=analogRead(pin);
  analoginput=map(analoginput, 10, 1013, 99, 0);
  return analoginput;
}

void verificar_joystick_mov_horizontal(){
  get_joystick_comand(A7);
  if(E0){
    mov_der=0;
    mov_izq=0;
  }
  if(E0 && pul_act && dir_act){
    E0=0;
    E1=1;
    pulso=1;
  }
  if(E0 && pul_act && !dir_act){
    E0=0;
    E1=1;
    pulso=0;
  }
  if(E1 && !pul_act && !dir_act){
    E1=0;
    E2=1;
  }
  if(E2){
    E2=0;
    E0=1;
    if(pulso){
      mov_der=1;
    }else{
      mov_izq=1;
    }
  }
}

void verificar_joystick_enter(){
  if(EW0){
    enter=0;
  }
  if(EW0 && !digitalRead(A3)){
    EW0=0;
    EW1=1;
  }
  if(EW1 && digitalRead(A3)){
    EW1=0;
    EW2=1;
  }
  if(EW2){
    EW2=0;
    EW0=1;
    enter=1;
  }
}

void Actualizar_LCD(){

    if(enable_actualizar_lcd && (millis()-t_ultima_actualizacion_lcd)>=100){
      enable_actualizar_lcd=0;
      t_ultima_actualizacion_lcd=millis();
      lcd.clear();
      switch (profundidad_en_menu) {
      case 1:
            lcd.setCursor(0, 0);
            lcd.print(F("Trabajar con:"));
            lcd.setCursor(0, 1);
            lcd.print("<");
            lcd.setCursor(6, 1);
        if(posicion_en_el_menu==0){ //CP
            lcd.print(F("CP"));
        }
        if(posicion_en_el_menu==1){ //NCM  lcd.print(<       NCL    >");
            lcd.print(F("NCM"));
        }
        if(posicion_en_el_menu==2){ //NCL
            lcd.print(F("NCL"));
        }
        if(posicion_en_el_menu==3){ //Riel
            lcd.print(F("RIEL"));
        }
        if(posicion_en_el_menu==4){ //Robot
            lcd.print(F("ROBOT"));
        }
        lcd.setCursor(15, 1);
        lcd.print(">");
      break;
      case 2:
        if(posicion_en_el_menu==0){
            lcd.setCursor(0, 0);
            lcd.print(F("<Acciones>"));
            lcd.setCursor(11, 0);
            imprimir_LCD_nombre_de_trabajando_con();
            lcd.setCursor(0, 1);
            lcd.print(atras_joy); //"Atras[*]  <JOY>"
        }
        if(posicion_en_el_menu==1){
            lcd.setCursor(0, 0);
            lcd.print(F("<Info>"));
            lcd.setCursor(11, 0);
            imprimir_LCD_nombre_de_trabajando_con();
            lcd.setCursor(0, 1);
            lcd.print(atras_joy); //"Atras[*]  <JOY>"
        }
      break;
      case 3:
        //**************************************************************************************************************RECEPCIONES
        lcd.setCursor(0, 0);
        imprimir_LCD_nombre_de_trabajando_con();
        lcd.setCursor(6, 0);
            if(estado_recibido_CP==0){
              lcd.print(F("OK"));
            }
            if(estado_recibido_CP==1){
              lcd.print(F("BUSY"));
            }
            if(estado_recibido_CP==2){
              lcd.print(F("E/A"));
            }
            if(estado_recibido_CP==3){
              lcd.print(F("DESH"));
            }
            if(estado_recibido_CP==4){
              lcd.print(F("WORK"));
            }
            if(estado_recibido_CP==5){
              lcd.print(F("PE"));
            }
            if(estado_recibido_CP==6){
              lcd.print(F("DESC"));
            }
        lcd.setCursor(11, 0);
        lcd.print("R");
        lcd.setCursor(12, 0);
        lcd.print(error_recibido_CP);
        lcd.setCursor(14, 0);
        lcd.print("M");
        lcd.setCursor(15, 0);
        lcd.print(modo_recibido_CP);
        lcd.setCursor(0, 1);
            if(trabajando_con_act==0 || trabajando_con_act==1){  //CP
              lcd.print('C');
              lcd.setCursor(1, 1);
              if(informacion_Cables<8){
                lcd.print('0');
                lcd.setCursor(2, 1);
                if(informacion_Cables<4){
                  lcd.print('0');
                  lcd.setCursor(3, 1);
                }
                if(informacion_Cables<2){
                  lcd.print('0');
                  lcd.setCursor(4, 1);
                }
                lcd.print(String(informacion_Cables, BIN));
              }else{
                lcd.print(String(informacion_Cables, BIN));
              }
              lcd.setCursor(6, 1);
            }
            if(trabajando_con_act==2){ //NCM
              lcd.print('V');
              lcd.setCursor(1, 1);
              if(informacion_valvulas_NCM<8){
                lcd.print('0');
                lcd.setCursor(2, 1);
                if(informacion_valvulas_NCM<4){
                  lcd.print('0');
                  lcd.setCursor(3, 1);
                }
                if(informacion_valvulas_NCM<2){
                  lcd.print('0');
                  lcd.setCursor(4, 1);
                }
                lcd.print(String(informacion_valvulas_NCM, BIN));
              }else{
                lcd.print(String(informacion_valvulas_NCM, BIN));
              }
              lcd.setCursor(6, 1);
            }
            if(trabajando_con_act==3){ //NCL
              lcd.print('V');
              lcd.setCursor(1, 1);
              if(informacion_valvulas_NCL<8){
                lcd.print('0');
                lcd.setCursor(2, 1);
                if(informacion_valvulas_NCL<4){
                  lcd.print('0');
                  lcd.setCursor(3, 1);
                }
                if(informacion_valvulas_NCL<2){
                  lcd.print('0');
                  lcd.setCursor(4, 1);
                }
                lcd.print(String(informacion_valvulas_NCL, BIN));
              }else{
                lcd.print(String(informacion_valvulas_NCL, BIN));
              }
              lcd.setCursor(6, 1);
            }
         //lcd.print(Debug_Comunicacion);
        //**************************************************************************************************************
      break;
      case 4:
        lcd.setCursor(0, 0);
        lcd.print(F("Cambiar Modo"));
        lcd.setCursor(0, 1);
        if(pulsacion=='0'){
          num_CP_selecionado=1;
        }
        if(pulsacion=='1'){
          num_CP_selecionado=2;
        }
        if(num_CP_selecionado==1 || num_CP_selecionado==2){
            if(num_CP_selecionado==1){
              lcd.print(F("0 - Manual?"));
            }
            if(num_CP_selecionado==2){
              lcd.print(F("1 - Auto?"));
            }
        }else{
            lcd.print(F("0=MANU | 1=AUTO"));
        }
      break;
      case 5:
      if(posicion_en_el_menu==0){ // velocidad
        velocidad=get_pot_val(A0);
        lcd.setCursor(0, 0);
        lcd.print(F("Vel NCM Eje: "));
        lcd.setCursor(13, 0);
        if(velocidad<10){
          lcd.print(" ");
          lcd.setCursor(14, 0);
          lcd.print(velocidad);
        }else{
          lcd.print(velocidad);
        }
        lcd.setCursor(0, 1);
        lcd.print(atras_joy);
      }
      if(posicion_en_el_menu==1){ // valvulas
        lcd.setCursor(0, 0);
        lcd.print(F("NCM ACTUADORES"));
        lcd.setCursor(0, 1);

        if(pulsacion=='4'){
          num_NCM_valvula_selecionado=1;
        }
        if(pulsacion=='5'){
          num_NCM_valvula_selecionado=2;
        }
        if(pulsacion=='6'){
          num_NCM_valvula_selecionado=3;
        }
        if(pulsacion=='B'){
          num_NCM_valvula_selecionado=4;
        }
        if(pulsacion=='7'){
          num_NCM_valvula_selecionado=5;
        }
        if(pulsacion=='8'){
          num_NCM_valvula_selecionado=6;
        }
        if(pulsacion=='9'){
          num_NCM_valvula_selecionado=7;
        }
        if(pulsacion=='C'){
          num_NCM_valvula_selecionado=8;
        }   
        
        if(num_NCM_valvula_selecionado>0 && num_NCM_valvula_selecionado<9){
          if(num_NCM_valvula_selecionado==1){
            lcd.print(F("ABRIR  Puerta?"));
          }
          if(num_NCM_valvula_selecionado==2){
            lcd.print(F("OFF - Agarre?"));
          }
          if(num_NCM_valvula_selecionado==3){
            lcd.print(F("OFF - Cooler?"));
          }
          if(num_NCM_valvula_selecionado==4){
            lcd.print(F("OFF - Spindle?"));
          }
          if(num_NCM_valvula_selecionado==5){
            lcd.print(F("CERRAR Puerta?"));
          }
          if(num_NCM_valvula_selecionado==6){
            lcd.print(F("ON  - Agarre?"));
          }
          if(num_NCM_valvula_selecionado==7){
            lcd.print(F("ON  - Cooler?"));
          }
          if(num_NCM_valvula_selecionado==8){
            lcd.print(F("ON  - Spindle?"));
          }     
        }else{
          lcd.print(F("456B=OFF 789C=ON"));
        }
      }
      if(posicion_en_el_menu==2){ // eje
        lcd.setCursor(0, 0);
        lcd.print(F("Selec EJE NCM"));
        lcd.setCursor(0, 1);

        if(pulsacion=='1'){
          num_NCM_eje_selecionado=1;
        }
        if(pulsacion=='2'){
          num_NCM_eje_selecionado=2;
        }
        if(pulsacion=='3'){
          num_NCM_eje_selecionado=3;
        }
        
        if(num_NCM_eje_selecionado>0 && num_NCM_eje_selecionado<4){
          if(enable_PUL_DIR_OUTPUT){
              if(num_NCM_eje_selecionado==1){
                lcd.print(F("<Joystick> mov X"));
              }
              if(num_NCM_eje_selecionado==2){
                lcd.print(F("<Joystick> mov Y"));
              }
              if(num_NCM_eje_selecionado==3){
                lcd.print(F("<Joystick> mov Z"));
              }
          }else{
              if(num_NCM_eje_selecionado==1){
                lcd.print(F("1 -mov X?"));
              }
              if(num_NCM_eje_selecionado==2){
                lcd.print(F("2 -mov Y?"));
              }
              if(num_NCM_eje_selecionado==3){
                lcd.print(F("3 -mov Z?"));
              }
          }         
        }else{
          lcd.print(F("1=X|2=Y|3=Z"));
        }
      }
      break;
      case 6:
      if(posicion_en_el_menu==0){ // velocidad
        velocidad=get_pot_val(A0);
  
        lcd.setCursor(0, 0);
        lcd.print(F("Vel Mov Eje: "));
        lcd.setCursor(13, 0);
        if(velocidad<10){
          lcd.print(" ");
          lcd.setCursor(14, 0);
          lcd.print(velocidad);
        }else{
          lcd.print(velocidad);
        }
        lcd.setCursor(0, 1);
        lcd.print(atras_joy+"  NCL");
      }
      if(posicion_en_el_menu==1){ // valvulas
  
        lcd.setCursor(0, 0);
        lcd.print(F("NCL ACTUADORES"));
        lcd.setCursor(0, 1);

        if(pulsacion=='4'){
          num_NCL_valvula_selecionado=1;
        }
        if(pulsacion=='5'){
          num_NCL_valvula_selecionado=2;
        }
        if(pulsacion=='6'){
          num_NCL_valvula_selecionado=3;
        }
        if(pulsacion=='B'){
          num_NCL_valvula_selecionado=4;
        }
        if(pulsacion=='7'){
          num_NCL_valvula_selecionado=5;
        }
        if(pulsacion=='8'){
          num_NCL_valvula_selecionado=6;
        }
        if(pulsacion=='9'){
          num_NCL_valvula_selecionado=7;
        }
        if(pulsacion=='C'){
          num_NCL_valvula_selecionado=8;
        }  
        
        if(num_NCL_valvula_selecionado>0 && num_NCL_valvula_selecionado<9){
          if(num_NCL_valvula_selecionado==1){
            lcd.print(F("ABRIR Puerta?"));
          }
          if(num_NCL_valvula_selecionado==2){
            lcd.print(F("OFF - Agarre?"));
          }
          if(num_NCL_valvula_selecionado==3){
            lcd.print(F("OFF - Cooler?"));
          }
          if(num_NCL_valvula_selecionado==4){
            lcd.print(F("OFF - Spindle?"));
          }
          if(num_NCL_valvula_selecionado==5){
            lcd.print(F("CERRAR  Puerta?"));
          }
          if(num_NCL_valvula_selecionado==6){
            lcd.print(F("ON  - Agarre?"));
          }
          if(num_NCL_valvula_selecionado==7){
            lcd.print(F("ON  - Cooler?"));
          }
          if(num_NCL_valvula_selecionado==8){
            lcd.print(F("ON  - Spindle?"));
          }
        }else{
          lcd.print(F("456B=OFF 789C=ON"));
        }
      }
      if(posicion_en_el_menu==2){ // eje
  
        lcd.setCursor(0, 0);
        lcd.print(F("Selec EJE NCL"));
        lcd.setCursor(0, 1);
        if(pulsacion=='1'){
          num_NCL_eje_selecionado=1;
        }
        if(pulsacion=='2'){
          num_NCL_eje_selecionado=2;
        }
        if(num_NCL_eje_selecionado>0 && num_NCL_eje_selecionado<3){
          if(enable_PUL_DIR_OUTPUT){
              if(num_NCL_eje_selecionado==1){
                lcd.print(F("<Joystick> mov X"));
              }
              if(num_NCL_eje_selecionado==2){
                lcd.print(F("<Joystick> mov Y"));
              }
          }else{
              if(num_NCL_eje_selecionado==1){
                lcd.print(F("1 -mov X?"));
              }
              if(num_NCL_eje_selecionado==2){
                lcd.print(F("2 -mov Y?"));
              }
          }
        }else{
          lcd.print(F("1=X|2=Y"));
        }
      }
      break;
      case 7:
        lcd.setCursor(0, 0);
        lcd.print(F("MOVER RIEL"));
        lcd.setCursor(0, 1);

        if(pulsacion=='0'){
          num_RIEL_selecionado=1;
        }
        if(pulsacion=='1'){
          num_RIEL_selecionado=2;
        }
        
        if(num_RIEL_selecionado==1 || num_RIEL_selecionado==2){
          if(num_RIEL_selecionado==1){
            lcd.print(F("0 - NO MOVER?"));
          }
          if(num_RIEL_selecionado==2){
            lcd.print(F("1 - MOVER?"));
          }
        }else{
          lcd.print(F("0=NO | 1=SI"));
        }
      break;
      case 8:
        lcd.setCursor(0, 0);
        lcd.print(F("MOVER ROBOT A:"));
        lcd.setCursor(0, 1);
        if(isdigit(pulsacion)){
          num_ROBOT_selecionado=String(pulsacion).toInt();    
        }
        lcd.print(num_ROBOT_selecionado);
        lcd.setCursor(3, 1);
        lcd.print(F("POSICION?"));
      break;
      default:
      break;
    }
  }
  
}

void imprimir_LCD_nombre_de_trabajando_con(){
      if(trabajando_con_act==0){  //CP
        lcd.print(F("CP"));
      }
      if(trabajando_con_act==2){  //NCM
        lcd.print(F("NCM"));
      }
      if(trabajando_con_act==3){  //NCL
        lcd.print(F("NCL"));
      }
      if(trabajando_con_act==4){  //RIEL
        lcd.print(F("RIEL"));
      }
      if(trabajando_con_act==5){  //ROBOT
        lcd.print(F("ROBOT"));
      }
}

void validar_enter(){
    switch (profundidad_en_menu) {
    case 1:
      if(posicion_en_el_menu==0){
        trabajando_con_act=0;
        enviar_a_CP(F("T0"));
      }
      if(posicion_en_el_menu==1){
        trabajando_con_act=2;
        enviar_a_CP(F("T2"));
      }
      if(posicion_en_el_menu==2){
        trabajando_con_act=3;
        enviar_a_CP(F("T3"));
      }
      if(posicion_en_el_menu==3){
        trabajando_con_act=4;
        enviar_a_CP(F("T4"));
      }
      if(posicion_en_el_menu==4){
        trabajando_con_act=5;
        enviar_a_CP(F("T5"));
      }
      profundidad_en_menu=2;
      posicion_en_el_menu=0;
    break;
    case 2:
      if(posicion_en_el_menu==0){
            if(trabajando_con_act==0){ //CP
              profundidad_en_menu=4;
              //num_CP_selecionado=0;
            }
            if(trabajando_con_act==2){ //NCM
              profundidad_en_menu=5;
              //num_NCM_valvula_selecionado=0;
              //num_NCM_eje_selecionado=0;
            }
            if(trabajando_con_act==3){ //NCL
              profundidad_en_menu=6;
              //num_NCL_valvula_selecionado=0;
              //num_NCL_eje_selecionado=0;
            }
            if(trabajando_con_act==4){ //Riel
              profundidad_en_menu=7;
              ///num_RIEL_selecionado=0;
            }
            if(trabajando_con_act==5){ //Robot
              profundidad_en_menu=8;
            }
            posicion_en_el_menu=0;
      }else{
        enable_enviar_peticiones_segun_trajando_con=1;
        profundidad_en_menu=3;
        posicion_en_el_menu=0;
      }
    break;
    case 3:
      enable_enviar_peticiones_segun_trajando_con=1;
    break;
    case 4:
      enable_enviar_accion_CP=1;
    break;
    case 5:
      enable_enviar_accion_NCM=1;
    break;
    case 6:
      enable_enviar_accion_NCL=1;
    break;
    case 7:
      enable_enviar_accion_RIEL=1;
    break;
    case 8:
      enable_enviar_accion_ROBOT=1;
    break;
    default:
    break;
  }
}

void validar_atras(){
    if(enable_PUL_DIR_OUTPUT){
        enable_PUL_DIR_OUTPUT=0;
    }else{
        posicion_en_el_menu=0;
        switch (profundidad_en_menu) {
        case 1:
          profundidad_en_menu=1;
        break;
        case 2:
          profundidad_en_menu=1;
        break;
        case 3:
          profundidad_en_menu=2;
        break;
        case 4:
          profundidad_en_menu=2;
        break;
        case 5:
          profundidad_en_menu=2;
          //enable_PUL_DIR_OUTPUT=0;
        break;
        case 6:
          profundidad_en_menu=2;
          //enable_PUL_DIR_OUTPUT=0;
        break;
        case 7:
          profundidad_en_menu=2;
          //enable_PUL_DIR_OUTPUT=0;
        break;
        case 8:
          profundidad_en_menu=2;
          //enable_PUL_DIR_OUTPUT=0;
        break;
        default:
        break;
      }
  }
}

void validar_posicion_menu(){
  if(!enable_PUL_DIR_OUTPUT){ //No se Movera
        switch (profundidad_en_menu) {
        case 1:
          if(posicion_en_el_menu>=0 && posicion_en_el_menu<4){ //0a4 5 posiciones
            posicion_en_el_menu+=1;
          }else{
            posicion_en_el_menu=0;
          }
        break;
        case 2:
          if(posicion_en_el_menu>=0 && posicion_en_el_menu<1){ //0a1 2 posiciones
            posicion_en_el_menu+=1;
          }else{
            posicion_en_el_menu=0;
          }
        break;
        case 3:
            posicion_en_el_menu=0;
        break;
        case 4:
            posicion_en_el_menu=0;
        break;
        case 5:
          if(posicion_en_el_menu>=0 && posicion_en_el_menu<2){ //0a2 2 posiciones
            posicion_en_el_menu+=1;
          }else{
            posicion_en_el_menu=0;
          }
        break;
        case 6:
          if(posicion_en_el_menu>=0 && posicion_en_el_menu<2){ //0a2 2 posiciones
            posicion_en_el_menu+=1;
          }else{
            posicion_en_el_menu=0;
          }
        break;
        case 7:
          posicion_en_el_menu=0;
        break;
        case 8:
    //      if(posicion_en_el_menu>=0 && posicion_en_el_menu<1){ //0a1 2 posiciones
    //        posicion_en_el_menu+=1;
    //      }else{
    //        posicion_en_el_menu=0;
    //      }
          posicion_en_el_menu=0;
        break;
        default:
        break;
      }
  }
}

void enviar_peticiones_segun_trabajando(){
    if(millis()-t_ultima_envio_a_CP>20){
      t_ultima_envio_a_CP=millis();
        if(EPP0){
          enviar_a_CP(F("PE"));
          EPP0=0;EPP1=1;
        }
        if(EPP1){
          enviar_a_CP(F("PR"));
          EPP1=0;EPP2=1;
        }
        if(EPP2){
          enviar_a_CP(F("PM"));
          EPP2=0;EPP3=1;
        }
        if(EPP3){
          enviar_a_CP(F("PC"));
          EPP3=0;EPP4=1;
        }
        if(EPP4){
          enviar_a_CP(F("PV"));
          EPP4=0;EPP5=1;
        }
        if(EPP5){
          EPP5=0;EPP0=1;
          //enable_actualizar_lcd=1;
          enable_enviar_peticiones_segun_trajando_con=0;
        }
    }
}

void Recepcion_Dato_MANDO(){
  char c1 = Serial.read();
  MANDO.rev=1;
  Buscar_comandos_MANDO(c1);
  if(MANDO.rev) grafcet_MANDO(c1);
}
void Buscar_comandos_MANDO(char c){ // Todos los comandos que se puede enviar el MANDO
  if(MANDO.enviando_comando_tipo==0){
    MANDO.rev=0;
      switch (c) {
        case 'P': //peticion_recibida
          MANDO.enviando_comando_tipo=1;
          MANDO.EP1=1;
        break;
        case 'T': //trabajo
          MANDO.enviando_comando_tipo=2;
          MANDO.ET1=1;
        break;
        case 'E': // estado_recibida
          MANDO.enviando_comando_tipo=3;
          MANDO.EE1=1;
        break;
        case 'R': //error
          MANDO.enviando_comando_tipo=4;
          MANDO.ER1=1;
        break;
//        case 'A': //accion_recibida
//          MANDO.enviando_comando_tipo=5;
//          MANDO.EA1=1;
//        break;
//        case 'S': //velocidad
//          MANDO.enviando_comando_tipo=6;
//          MANDO.ES1=1;
//        break;
        case 'V': //valvulas
          MANDO.enviando_comando_tipo=7;
          MANDO.EV1=1;
        break;
        case 'C': //cable
          MANDO.enviando_comando_tipo=8;
          MANDO.EC1=1;
        break;
        case 'M': //modo
          MANDO.enviando_comando_tipo=9;
          MANDO.EM1=1;
        break;
        default:
          MANDO.rev=1;
          SERIAL_PRINT(F("Error No se encuentra"));
          SERIAL_PRINTLN(c);
        break;
      }
  }
}

void grafcet_MANDO(char c){
    switch (MANDO.enviando_comando_tipo) {
    case 1:
      grafcet_peticiones_MANDO(c); //depende con quien se trabaje
    break;
    case 2:
      grafcet_trabajos_MANDO(c); //recibe el trabajo actual
    break;
    case 3:
      grafcet_estados_MANDO(c); //Recibe los estados de todos los modulos
    break;
    case 4:
      grafcet_errores_MANDO(c);  //Recibe los estados de todos los modulos
    break;
//    case 5:
//      grafcet_acciones_MANDO(c);  //depende con quien se trabaje
//    break;
//    case 6:
//      grafcet_velocidad_MANDO(c); // solo funciona cuando se trabaja con NCL y NCM
//    break;
    case 7:
      grafcet_valvulas_MANDO(c);
    break;
    case 8:
      grafcet_cables_MANDO(c);
    break;
    case 9:
      grafcet_modo_MANDO(c);
    break;
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
//      case 'M': //Modo_actual
//        MANDO.peticion_recibida=2;
//      break;
      case 'A': //Alertas
        MANDO.peticion_recibida=3;
      break;
      case 'R': //Errores
        MANDO.peticion_recibida=4;
      break;
//      case 'V': //Estado_Electrovalvulas
//        MANDO.peticion_recibida=5;
//      break;
//      case 'C': //Cables desconectados
//        MANDO.peticion_recibida=6;
//      break;      
      default:
        MANDO.rev=1;
        MANDO.EP1=0;
        MANDO.EP2=0;
        MANDO.enviando_comando_tipo=0;
        SERIAL_PRINT(F("Error_P: "));
        SERIAL_PRINTLN(c);
      break;
    }
  }
  if(MANDO.EP2 && MANDO.rev){
    MANDO.EP2=0;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      MANDO.enable_enviado_peticion_completo=1;
      MANDO.recibido_nuevo_comando=1;
      SERIAL_PRINT(F("MANDO.peticion_recibida: "));
      SERIAL_PRINTLN(MANDO.peticion_recibida);
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
        MANDO.ET1=0;
        MANDO.ET2=0;
        MANDO.enviando_comando_tipo=0;
        SERIAL_PRINT(F("Error_T: "));
        SERIAL_PRINTLN(c);
      break;
    }
  }
  if(MANDO.ET2 && MANDO.rev){
    MANDO.ET2=0;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      MANDO.enviado_trabajo_completo=1;
      MANDO.recibido_nuevo_comando=1;
      SERIAL_PRINT(F("Trabajo: "));
      SERIAL_PRINTLN(MANDO.trabajar_con_recibida);
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
          MANDO.EE1=0;
          MANDO.EE2=0;
          MANDO.enviando_comando_tipo=0;
          SERIAL_PRINT(F("Error_E: "));
          SERIAL_PRINTLN(c);
        break;
      }
    }
    if(MANDO.EE2 && MANDO.rev){
      MANDO.EE2=0;
      MANDO.enviando_comando_tipo=0;
      if(c==10 || c==13){
        MANDO.enviado_estado_completo=1;
        MANDO.recibido_nuevo_comando=1;
        SERIAL_PRINT(F("MANDO.estado_recibida: "));
        SERIAL_PRINTLN(MANDO.estado_recibida);
        MANDO.rev=0;
      }
    }
}

void grafcet_errores_MANDO(char c){
  if(MANDO.ER1){
    MANDO.ER1=0;
    MANDO.ER2=1;
    MANDO.rev=0;
//    switch (c) {
//      case '0': // no error
//        MANDO.error_recibida=0;
//      break;
//      case '1': // Joystick_error
//        MANDO.error_recibida=1;
//      break;
//      default:
//        MANDO.rev=1;
//        MANDO.ER0=1;
//        MANDO.ER1=0;
//        MANDO.ER2=0;
//        MANDO.enviando_comando_tipo=0;
//        SERIAL_PRINT(F("Error_E: "));
//        SERIAL_PRINTLN(c);
//      break;
//    }
    if(isdigit(c)){
      MANDO.error_recibida=String(c).toInt();
    }else{
      MANDO.rev=1;
      MANDO.ER1=0;
      MANDO.ER2=0;
      MANDO.enviando_comando_tipo=0;
      SERIAL_PRINT(F("Error_E: "));
      SERIAL_PRINTLN(c);
    }
  }
  if(MANDO.ER2 && MANDO.rev){
    MANDO.ER2=0;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      MANDO.enviado_error_completo=1;
      MANDO.recibido_nuevo_comando=1;
      SERIAL_PRINT(F("error: "));
      SERIAL_PRINTLN(MANDO.error_recibida);
      MANDO.rev=0;
    }
  }
}

void grafcet_valvulas_MANDO(char c){
  if(MANDO.EV1){
    MANDO.EV1=0;
    MANDO.EV2=1;
    MANDO.rev=0;
    if(isdigit(c)){
      MANDO.valvulas_recibida=String(c).toInt()*10;
    }else{
      MANDO.rev=1;
      MANDO.EV2=0;
      MANDO.valvulas_recibida=0;
      MANDO.enviando_comando_tipo=0;
      SERIAL_PRINT(F("Error_valvulas_#1: "));
      SERIAL_PRINTLN(c);
    }
  }
  if(MANDO.EV2 && MANDO.rev){
    MANDO.EV2=0;
    MANDO.EV3=1;
    MANDO.rev=0;
    if(isdigit(c)){
      MANDO.valvulas_recibida+=String(c).toInt();
    }else{
      MANDO.rev=1;
      MANDO.EV3=0;
      MANDO.valvulas_recibida=0;
      MANDO.enviando_comando_tipo=0;
      SERIAL_PRINT(F("Error_valvulas_#2: "));
      SERIAL_PRINTLN(c);
    }
  }
  if(MANDO.EV3 && MANDO.rev){
    MANDO.EV3=0;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      if(MANDO.valvulas_recibida<16){
          MANDO.enviado_valvulas_completo=1;
          MANDO.recibido_nuevo_comando=1;
          SERIAL_PRINT(F("MANDO.valvulas_recibida"));
          SERIAL_PRINTLN(MANDO.valvulas_recibida);
          MANDO.rev=0;
      }else{
        SERIAL_PRINT(F("Valvu_fuera_rango: "));
        SERIAL_PRINTLN(MANDO.valvulas_recibida);
      }
    }else{
      SERIAL_PRINT(F("falto_enter_V: "));
      SERIAL_PRINTLN(c);
    }
  }
}

void grafcet_cables_MANDO(char c){
  if(MANDO.EC1){
    MANDO.EC1=0;
    MANDO.EC2=1;
    MANDO.rev=0;
    if(isdigit(c)){
      MANDO.cables_recibida=String(c).toInt()*10;
    }else{
      MANDO.rev=1;
      MANDO.EC2=0;
      MANDO.cables_recibida=0;
      MANDO.enviando_comando_tipo=0;
      SERIAL_PRINT(F("Error_cables1: "));
      SERIAL_PRINTLN(c);
    }
  }
  if(MANDO.EC2 && MANDO.rev){
    MANDO.EC2=0;
    MANDO.EC3=1;
    MANDO.rev=0;
    if(isdigit(c)){
      MANDO.cables_recibida+=String(c).toInt();
    }else{
      MANDO.rev=1;
      MANDO.EC3=0;
      MANDO.cables_recibida=0;
      MANDO.enviando_comando_tipo=0;
      SERIAL_PRINT(F("Error_cables2: "));
      SERIAL_PRINTLN(c);
    }
  }
  if(MANDO.EC3 && MANDO.rev){
    MANDO.EC3=0;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      if(MANDO.cables_recibida<32){
        MANDO.enviado_cables_completo=1;
        MANDO.recibido_nuevo_comando=1;
        SERIAL_PRINT(F("MANDO.cables_recibida"));
        SERIAL_PRINTLN(MANDO.cables_recibida);
        MANDO.rev=0;
      }else{
        SERIAL_PRINT(F("Cables_fuera_rango: "));
        SERIAL_PRINTLN(MANDO.cables_recibida);
      }
    }else{
      SERIAL_PRINT(F("falto_enter_C: "));
      SERIAL_PRINTLN(c);
    }
  }
}

void grafcet_modo_MANDO(char c){
  if(MANDO.EM1){
    MANDO.EM1=0;
    MANDO.EM2=1;
    MANDO.rev=0;
    switch (c) {
      case '0': // MODO ACTUAL DE CP MANUAL
        MANDO.modo_recibida=0;
      break;
      case '1': // MODO ACTUAL DE CP AUTOMATICO
        MANDO.modo_recibida=1;
      break;
      default:
        MANDO.rev=1;
        MANDO.EM1=0;
        MANDO.EM2=0;
        MANDO.enviando_comando_tipo=0;
        SERIAL_PRINT(F("modo_E: "));
        SERIAL_PRINTLN(c);
      break;
    }
  }
  if(MANDO.EM2 && MANDO.rev){
    MANDO.EM2=0;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      MANDO.enviado_modo_completo=1;
      MANDO.recibido_nuevo_comando=1;
      SERIAL_PRINT(F("modo: "));
      SERIAL_PRINTLN(MANDO.modo_recibida);
      MANDO.rev=0;
    }
  }
}

void Respuesta_a_comandos_MANDO(){
        if(MANDO.enviado_trabajo_completo){
          MANDO.enviado_trabajo_completo=0;
          trabajando_con_act=MANDO.trabajar_con_recibida;
          posicion_en_el_menu=0;
          profundidad_en_menu=1;
          MANDO.trabajar_con_recibida=0;
          
        }else{
            if(MANDO.enable_enviado_peticion_completo){// Respuesta a las peticiones del MANDO
                  MANDO.enable_enviado_peticion_completo=0;
                  if(MANDO.peticion_recibida==1){
                    SERIAL_PRINTLN(F("E0"));////////////////////////////////////////////////////////////////////////////////
                  }else{
                    SERIAL_PRINTLN(F("R0"));///////////////////////////////////////////////////////////////////////////////
                  }
                  MANDO.peticion_recibida=0;
            }
            if(MANDO.enviado_estado_completo){// Respuesta a las peticiones del MANDO
                  MANDO.enviado_estado_completo=0;
                  estado_recibido_CP=MANDO.estado_recibida;
                  MANDO.estado_recibida=0;
            }
            if(MANDO.enviado_error_completo){// Respuesta a las peticiones del MANDO
                  MANDO.enviado_error_completo=0;
                  error_recibido_CP=MANDO.error_recibida;
                  MANDO.error_recibida=0;
            }
            if(MANDO.enviado_valvulas_completo){// Respuesta a las peticiones del MANDO
                  MANDO.enviado_valvulas_completo=0;
                  if(trabajando_con_act==2){  //NCM
                    informacion_valvulas_NCM=MANDO.valvulas_recibida;
                  }
                  if(trabajando_con_act==3){  //NCL
                    informacion_valvulas_NCL=MANDO.valvulas_recibida;
                  }
                  MANDO.valvulas_recibida=0;
            }
            if(MANDO.enviado_cables_completo){// Respuesta a las peticiones del MANDO
                  MANDO.enviado_cables_completo=0;
                  informacion_Cables=MANDO.cables_recibida;
                  MANDO.cables_recibida=0;
            }
            if(MANDO.enviado_modo_completo){// Respuesta a las peticiones del MANDO
                  MANDO.enviado_modo_completo=0;
                  modo_recibido_CP=MANDO.modo_recibida;
                  MANDO.modo_recibida=0;
            }
        }


}

