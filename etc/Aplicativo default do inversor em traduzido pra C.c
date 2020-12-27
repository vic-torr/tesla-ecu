#include <stdlib.h>
#include <stdio.h>


typedef bool unsigned char;
typedef enum {false,true};

bool desabilita, veiculoLigado, driveHabilitado, delayHabilitado,
      generalEnabled, runStop, velNula, IX1_frente, semFNR IX4_freio,
      chaveFrente, chaveFreio, habilitaSomAlarme, habilitaOscilacao, erroFrente
      modoAceleracao, pedalAcionado, freio, modoNeutro;

unsigned int pedalAcelerador1, pedalAcelerador2, desaceleracao;
float velocidade,aceleracao, desaceleracao, fatorDesacel, refVel;

int main(){
// vel_real_rpm = motorSpeed (sw3304)
if( vel_real_rpm < 20 ) velNula = 1;

pedalAcel = pedal_in;
if (pedalAcel > 0.05) pedalAcionado = true;


chaveFrente = chaveFrenteIn && velNula;
neutro = !chaveFrente;
freio = freioIn;
desaceleracao = 10 * (Desaceleracao_frente + 1);
modoAceleracao = pedalAcionado && chaveFrente && !erroFrente && !freio && !neutro;

//fator de desaceleracao: reducao na referencia de velocidade, 0-1.5 * desacel maxima
// 2000 < vel < 6000 -> nivel 4 -> fatorDesacel = 1.5 * desaceleracao
// 1000 < vel < 2000 -> nivel 3 -> fatorDesacel = 0.8 * desaceleracao
// 500 < vel < 1000 -> nivel 2 -> fatorDesacel = 0.5 * desaceleracao
// 250 < vel < 500 -> nivel 1 -> fatorDesacel = 0.3 * desaceleracao
// 100 < vel < 250 -> nivel 0 -> fatorDesacel = 0.1 * desaceleracao
// vel < 100 -> nivel 00 -> fatorDesacel = 0.05 * desaceleracao

// usando regressao linar, erro maximo de 7%:
fatorDesacel = 0.0007899 * vel - 0.058196

// vel_real_rpm = ((vel_max_kmph/3.6) / ((diametroPneu_mm/1000)*2pi))*60* rel_transmissao
vel_acel1 = vel_real_rpm + aceleracao;
// 0 < vel_acel2 < vel_max_rpm  -> velAcel3
if ( vel acel2 < 0 && vel_acel2 < vel_max_rpm ) velAcel3 = vel_acel2;
ref_vel = vel_acel3;

velFrenagem = vel_real_rpm - velFreio;
// 0 < vel_acel2 < vel_max_rpm  -> vel_frenagem
if ( !velNula && !modoAceleracao && !freio ) refVel = 0;
if ( freio ) refVel = vel_real_rpm - pedal_freio;

// vel_real_rpm < 1100   -> vel1
// 1100 < vel_real_rpm < 1500   -> vel2
// ...
// 6000 < vel_real_rpm < 9000   -> vel9

if ( modoAceleracao && ( vel1 || vel2 || vel3 )) Max_torque = Torque_max;
if ( modoAceleracao && vel 4 ) Max_torque = 120;
// ...
if ( modoAceleracao && vel 9 ) Max_torque = 30;
if ( !modoAceleracao ) max_torque = Torque_max;

correntePico = 400, correnteNominal = 200;
//InMotor = P0401 ( 0 a 1.3 Inom );
fatorImax = (Imax) / (In_motor/100);
// 0 < Torque_max/100 < fatorImax
refTorque = torqueMax * 1000;
if ( !desabilita && driveHabilitado ) torqueFrente = refTorque;

// fluxo de magnetizacao
if ( desabilita || velNula ) fluxoMaximo = 1;
if ( desabilita && velNula) fluxoMaximo = 120;

//Parada em rampa - transmite pouco torque
 if ( hab_parada_rampa && velNula && !modoAceleracao && !freio && !rampa ){
  delay(tempo_parada_rampa);
  desabilita = 1;
  aux_rampa = 1;
}
if ( desabilita && !velNula && driveHabilitado ) refTorque = 1;
if( !driveHabilitado ) refTorque = 0;


// dataLogger
// envia: driveHabilitado, chaveFrente, chaveFreio




}
