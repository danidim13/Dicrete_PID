#ifndef _PID_HPP_
#define _PID_HPP_

class PID {

    public:
        /**
         * Crea un nuevo controlador PID.
         * @param Kp   Ganancia proporcial.
         * @param Ti   Tiempo integral.
         * @param Td   Tiempo derivativo.
         * @param Alfa Orden del filtro.
         * @param Beta Beta.
         * @param Ts   Tiempo de muestreo.
         */
        PID (double Kp, double Ti, double Td, double Alfa, double Beta, double Ts);

        /**
         * Destructor por defecto.
         */
        virtual ~PID ();

        /**
         * Cambia los limites del esfuerzo de control.
         * @param min Valor mínimo.
         * @param max Valor máximo.
         */
        void SetMaxEffort(double min, double max);

        /**
         * Actualiza el valor deseado
         * @param Ref Nuevo
         */
        void UpdateSetpoint(double Ref);

        /**
         * Calcula la salida del controlador y actualiza las variables.
         * @param  Yk_read La lectura del valor actual de la variable controlada.
         * @return         La acción de control.
         */
        double Step(double Yk_read);

        /**
         * Calcula la salida del controlador y actualiza las variables,
         * con antiwindup.
         * @param  Yk_read La lectura del valor actual de la variable controlada.
         * @return         La acción de control.
         */
        double StepAntiWind(double Yk_read);

        /**
         * Actualiza las variables internas del controlador en modo manual.
         * @param Yk_read La lectura del valor actual de la variable controlada.
         * @param Uk_man  La acción que se está pasando directamente a la planta.
         */
        void Bypass(double Yk_read, double Uk_man);

        double GetIk();

    private:
        /* Parametros del controlador */
        double m_Kp;
        double m_Ti;
        double m_Td;
        double m_Alfa;
        double m_Beta;
        double m_Ts;
        double m_Ta;

        double m_MaxEff;
        double m_MinEff;

        /* Variables y estados internos del controlador */
        double m_Pk;
        double m_Ik_plus1;
        double m_Ik;
        double m_Dk;
        double m_Dk_min1;

        /* Entradas */
        double m_Refk;
        double m_Yk;
        double m_Yk_min1;

        /* Salidas */
        double m_Uk;
        double m_satUk;
        double m_antiwind;
};

#endif
