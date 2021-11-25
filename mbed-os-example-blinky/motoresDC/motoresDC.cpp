#include "motoresDC.h"
     void wait_ms(float ms){
    wait_us(ms * 1000 );
}
    MotoresDC::MotoresDC(PinName MI_velPin, 
                        PinName MI_s1Pin, 
                        PinName MI_s2Pin, 
                        PinName MD_velPin, 
                        PinName MD_s1Pin, 
                        PinName MD_s2Pin):_MI_vel(MI_velPin), _MI_s1(MI_s1Pin), _MI_s2(MI_s2Pin), _MD_vel(MD_velPin), _MD_s1(MD_s1Pin), _MD_s2(MD_s2Pin)
    {
        _R = 3.25;
        _L = 9.0;
        _MI_vel.period_ms(40);
        _MI_vel.write(0);
        _MD_vel.period_ms(40);
        _MD_vel.write(0);
    }
    
    void MotoresDC::setParams(float R, float L)
    {
        this->_R = R;
        this->_L = L;
    }
        
    void MotoresDC::conducirUniciclo(float v, float w)
    {
        float v_r = (2 * v + this->_L * w)/(2 * this->_R);
        float v_l = (2 * v - this->_L * w)/(2 * this->_R);
        
        this->motorIzq(v_l);
        this->motorDer(v_r);  
    }
    /* funciones base para conducir */
    void MotoresDC::_izqAdelante(float velocidad)
    {
        _MI_s1 = 1;
        _MI_s2 = 0;
        _MI_vel = abs(velocidad*5.0/9);   
        //_MI_vel = abs(velocidad);
    }
    
    void MotoresDC::_izqAtras(float velocidad)
    {
        _MI_s1 = 0;
        _MI_s2 = 1;
        _MI_vel = abs(velocidad*5.0/9);  
       // _MI_vel = abs(velocidad);
    }
    
    void MotoresDC::_derAdelante(float velocidad)
    {
        _MD_s1 = 1;
        _MD_s2 = 0;
        _MD_vel = abs(velocidad); 
    }
    
    void MotoresDC::_derAtras(float velocidad)  
    {
        _MD_s1 = 0;
        _MD_s2 = 1;
        _MD_vel = abs(velocidad);   
    }  
    
    /* funciones compuestas */
    /* funciones para detener los motores */
    void MotoresDC::detenerIzq(void)
    {
        _MI_s1 = 0;
        _MI_s2 = 0;
        _MI_vel = 0;
    }
    
    void MotoresDC::detenerDer(void)
    {
        _MD_s1 = 0;
        _MD_s2 = 0;
        _MD_vel = 0;
    }
    void MotoresDC::frenarIzq(void)
    {
        _MI_s1 = 1;
        _MI_s2 = 1;
        _MI_vel = 0;
    }
    void MotoresDC::frenarDer(void)
    {
        _MD_s1 = 1;
        _MD_s2 = 1;
        _MD_vel = 0;
    }
    void MotoresDC::detener(void)
    {
        detenerIzq();
        detenerDer();
    }
    void MotoresDC::frenar(void)
    {
        frenarIzq();
        frenarDer();
    }
    
    /* --------------- */
    
    /* funcion para conducir ambos motores en la misma direccion */
    void MotoresDC::conducir(float velocidad)
    {
        if(velocidad > 0)
        {
            _izqAdelante(velocidad);
            _derAdelante(velocidad);
        }
        else
        {
            _izqAtras(velocidad);
            _derAtras(velocidad);
        }
    }
    void MotoresDC::conducir(float velocidad, int duracion)
    {
        if(velocidad > 0)
        {
            _izqAdelante(velocidad);
            _derAdelante(velocidad);
        }
        else
        {
            _izqAtras(velocidad);
            _derAtras(velocidad);
        }
        wait_ms(duracion);
        detenerIzq();
        detenerDer();
    }
    /* funcion para conducir motores en sentido contrario */
    void MotoresDC::pivotar(float velocidad)
    {
        if(velocidad > 0)
        {
            _izqAdelante(velocidad);
            _derAtras(velocidad);
        }
        else
        {
            _izqAtras(velocidad);
            _derAdelante(velocidad);
        }
    }
    void MotoresDC::pivotar(float velocidad, int duracion)
    {
        if(velocidad > 0)
        {
            _izqAdelante(velocidad);
            _derAtras(velocidad);
        }
        else
        {
            _izqAtras(velocidad);
            _derAdelante(velocidad);
        }
        wait_ms(duracion);
        detenerIzq();
        detenerDer();
    }
        
    /* funciones para conducir motores independientemente */
    void MotoresDC::motorIzq(float velocidad)
    {
        if(velocidad > 0)
        {
            _izqAdelante(velocidad);
        }
        else
        {
            _izqAtras(velocidad);
        }
    }
    
    void MotoresDC::motorDer(float velocidad)
    {
        if(velocidad > 0)
        {
            _derAdelante(velocidad);
        }
        else
        {
            _derAtras(velocidad);
        }
    }
    /* funciones para conducir motores independientemente con duracion */
    void MotoresDC::motorIzq(float velocidad, int duracion)
    {
        if(velocidad > 0)
        {
            _izqAdelante(velocidad);
        }
        else
        {
            _izqAtras(velocidad);
        }
        wait_ms(duracion);
        detenerIzq();
    }
    void MotoresDC::motorDer(float velocidad, int duracion)
    {
        if(velocidad > 0)
        {
            _derAdelante(velocidad);
        }
        else
        {
            _derAtras(velocidad);
        }
        wait_ms(duracion);
        detenerDer();
    }
    
    
        
  