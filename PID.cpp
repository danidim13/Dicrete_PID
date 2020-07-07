#include "PID.hpp"
#include <math.h>

PID::PID(double Kp, double Ti, double Td, double Alfa, double Beta, double Ts) {
    m_Kp = Kp;
    m_Ti = Ti;
    m_Td = Td;
    m_Alfa = Alfa;
    m_Beta = Beta;
    m_Ts = Ts;

    if (Td > 0 && Ti > 0) {
        m_Ta = sqrt(Ti*Td);
    } else if (Ti > 0) {
        m_Ta = Ti;
    }

    m_Ik_plus1 = 0;
    m_Yk_min1 = 0;
    m_Dk_min1 = 0;

    m_MaxEff = 1;
    m_MinEff = -1;
}

PID::~PID() {

}

void PID::SetMaxEffort(double min, double max) {
    m_MaxEff = max;
    m_MinEff = min;
}

void PID::UpdateSetpoint(double Ref){
    m_Refk = Ref;
}

double PID::Step(double Yk_read) {
    m_Yk = Yk_read;

    m_Pk = m_Kp*(m_Beta*m_Refk - m_Yk);
    m_Ik = m_Ik_plus1;
    m_Dk = m_Dk_min1*(m_Alfa*m_Td)/(m_Alfa*m_Td + m_Ts) - (m_Yk - m_Yk_min1)*m_Kp*m_Td/(m_Alfa*m_Td + m_Ts);

    // Salida del controlador
    m_Uk = m_Pk + m_Dk + m_Ik;

    // Actualizar variables
    m_Ik_plus1 = m_Ik + (m_Refk - m_Yk)*m_Kp*m_Ts/m_Ti;
    m_Dk_min1 = m_Dk;
    m_Yk_min1 = m_Yk;
    return m_Uk;
}

double PID::StepAntiWind(double Yk_read) {
    m_Yk = Yk_read;

    m_Pk = m_Kp*(m_Beta*m_Refk - m_Yk);
    m_Ik = m_Ik_plus1;
    m_Dk = m_Dk_min1*(m_Alfa*m_Td)/(m_Alfa*m_Td + m_Ts) - (m_Yk - m_Yk_min1)*m_Kp*m_Td/(m_Alfa*m_Td + m_Ts);

    // Salida del controlador
    m_Uk = m_Pk + m_Dk + m_Ik;
    m_satUk = fmax(m_MinEff, fmin(m_MaxEff, m_Uk));
    m_antiwind = (m_satUk - m_Uk)/m_Ta;

    // Actualizar variables
    m_Ik_plus1 = m_Ik + (m_Refk - m_Yk)*m_Kp*m_Ts/m_Ti + m_antiwind;
    m_Dk_min1 = m_Dk;
    m_Yk_min1 = m_Yk;
    return m_satUk;
}

void PID::Bypass(double Yk_read, double Uk_man) {

    // Pk = Kp*(beta * Yk - yk) = Kp * yk (beta - 1);
    // Uk_man = Ik_1 + Pk --> Ik_1 = Uk_man - Pk
    
    double Uk_man_sat = fmax(m_MinEff, fmin(m_MaxEff, Uk_man));
    m_Ik_plus1 = Uk_man_sat - m_Kp*Yk_read*(m_Beta - 1);
    m_Dk_min1 = 0;
    m_Yk_min1 = Yk_read;
}

double PID::GetIk() {
    return m_Ik;
}

