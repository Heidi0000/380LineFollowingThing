
#ifndef MBED_MOTORESDC_H
#define MBED_MOTORESDC_H

#include "mbed.h"
/** 
*    Libreria para controlar un driver dual de motores DC
*    funciona con drivers como el L293D o el L298N
*    se necesitan 6 pines del microcontrolador
*    uno para la velocidad (PWM) y 2 para el sentido (digital)
*    esta librería esta basada en la libreria RedBot de Sparkfun   
*   Ejemplo:
*   @code
*   #include "mbed.h"
*   #include "motoresDC.h"
*   MotoresDC carro(PA_8, PB_12, PB_13, PA_9, PB_14, PB_15);
*   int main()
*   {
*       carro.conducir(0.5, 1000);  // adelante por 1 segundo a media velocidad
*       carro.conducir(-0.5, 1000); // atrás por 1 segundo a media velocidad
*       carro.pivotar(0.6, 400);    // pivota izquierda por 0.4 segundos a un 60% de la velocidad
*       carro.conducir(-0.6, 400);  // pivota derecha por 0.4 segundos a un 60% de la velocidad
*       carro.detener();
*   }
*   @endcode
*/
class MotoresDC
{
    public:
        /** Instancia la clase con los pines del driver
         * @param MI_vel Pin para el control de velocidad del motor izquierdo.
         * @param MI_s1 Primer pin de control de sentido para el motor izquierdo.
         * @param MI_s2 Segundo pin de control de sentido para el motor izquierdo.
         * @param MD_vel Pin para el control de velocidad del motor derecho.
         * @param MD_s1 Primer pin de control de sentido para el motor derecho.
         * @param MD_s2 Segundo pin de control de sentido para el motor derecho.
        */
        MotoresDC(PinName MI_vel, PinName MI_s1, PinName MI_s2, 
                    PinName MD_vel, PinName MD_s1, PinName MD_s2);
                    
        // metodos publicos //
        /** Configura los parámetros del uniciclo
        * @param R Radio en cm de las ruedas.
        * @param L Longitud del eje en cm.
        */
        void setParams(float R, float L);
        
        /** Método para conducir con el modelo de robot diferencial a uniciclo.
        * @param v velocidad lineal.
        * @param w velocidad angular.
        */
        void conducirUniciclo(float v, float w);
        
        /** Método para conducir ambos motores en la misma direccion 
        * @param velocidad valor de velocidad de ambos motores
        */
        void conducir(float velocidad);
        
        /** Método para conducir ambos motores en la misma direccion con una duración en ms 
        * @param velocidad valor de velocidad de ambos motores.
        * @param duración en milisegundos.
        */
        void conducir(float velocidad, int duracion);
        
        /** Método para conducir ambos motores en direcciones opuestas 
        * @param velocidad valor de velocidad de ambos motores (complementarias).
        */
        void pivotar(float velocidad);
        
        /** Método para conducir ambos motores en direcciones opuestas con una duración en ms 
        * @param velocidad valor de velocidad de ambos motores (complementarias).
        * @param duración en milisegundos.
        */
        void pivotar(float velocidad, int duracion);
        
        /* funciones para conducir motores independientemente */
        /** Conducir motor izquierdo 
        * @param velocidad valor de velocidad.
        */
        void motorIzq(float velocidad);
        
        /** Conducir motor izquierdo 
        * @param velocidad valor de velocidad.
        * @param duración en milisegundos.
        */
        void motorDer(float velocidad);
        
        /** Conducir motor derecho 
        * @param velocidad valor de velocidad.
        */
        void motorIzq(float velocidad, int duracion);
        
        /** Conducir motor derecho 
        * @param velocidad valor de velocidad.
        * @param duración en milisegundos.
        */
        void motorDer(float velocidad, int duracion);
        
        /* funciones para detener los motores */
        /** Detiene ambos motores */
        void detener(void);
        
        /** Frena ambos motores */
        void frenar(void);
        
        /** Detiene motor izquierdo */
        void detenerIzq(void);
        
        /** Detiene motor derecho */
        void detenerDer(void);
        
        /** Frena motor izquierdo */
        void frenarIzq(void);
        
        /** Frena motor derecho */
        void frenarDer(void);
        /* --------------- */
    // metodos y atributos privados //
    private:
    
        /** variables para conversión del robot diferencial a uniciclo */
        float _R;
        float _L;
        /* velocidad izquierda */
        PwmOut _MI_vel;
        /* sentido de giro */
        DigitalOut _MI_s1;  // motor izq
        DigitalOut _MI_s2;
        /* velocidad derecha */
        PwmOut _MD_vel;
        /* sentido de giro */
        DigitalOut _MD_s1;
        DigitalOut _MD_s2;  
        
        
        /* funciones */
        
        /** Conduce el motor izquierdo hacia adelante
        * @param velocidad valor de velocidad en porcentaje.
        */
        void _izqAdelante(float velocidad);
        
        /** Conduce el motor izquierdo hacia atras
        * @param velocidad valor de velocidad en porcentaje.
        */
        void _izqAtras(float velocidad);
        
        /** Conduce el motor derecho hacia adelante
        * @param velocidad valor de velocidad en porcentaje.
        */
        void _derAdelante(float velocidad);
        
        /** Conduce el motor derecho hacia atras
        * @param velocidad valor de velocidad en porcentaje.
        */
        void _derAtras(float velocidad);
        
};
#endif