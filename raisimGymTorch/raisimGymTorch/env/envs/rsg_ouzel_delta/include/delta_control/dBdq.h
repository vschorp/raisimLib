double t2 = std::cos(q1);
double t3 = std::cos(q2);
double t4 = std::cos(q3);
double t5 = std::sin(q1);
double t6 = std::sin(q2);
double t7 = std::sin(q3);
double t8 = l_p * l_p;
double t9 = std::sqrt(3.0);
double t10 = r_b / 2.0;
double t11 = r_p / 2.0;
double t12 = -t11;
double t13 = t9 * t10;
double t14 = t9 * t11;
A0(0, 0) = -qd1 * ((t2 * t2) * t8 * 2.0 + (t5 * t5) * t8 * 2.0 +
                   l_p * t5 * (z - l_p * t5) * 2.0 -
                   l_p * t2 * (r_b - r_p - x + l_p * t2) * 2.0);
A0(1, 1) = -qd2 * ((t3 * t3) * t8 * 2.0 + (t6 * t6) * t8 * 2.0 -
                   l_p * t3 * (t10 + t12 + x + (l_p * t3) / 2.0) +
                   l_p * t6 * (z - l_p * t6) * 2.0 +
                   l_p * t3 * t9 *
                       (t14 + y - (r_b * t9) / 2.0 - (l_p * t3 * t9) / 2.0));
A0(2, 2) = -qd3 * ((t4 * t4) * t8 * 2.0 + (t7 * t7) * t8 * 2.0 -
                   l_p * t4 * (t10 + t12 + x + (l_p * t4) / 2.0) +
                   l_p * t7 * (z - l_p * t7) * 2.0 -
                   l_p * t4 * t9 *
                       (t13 + y - (r_p * t9) / 2.0 + (l_p * t4 * t9) / 2.0));
