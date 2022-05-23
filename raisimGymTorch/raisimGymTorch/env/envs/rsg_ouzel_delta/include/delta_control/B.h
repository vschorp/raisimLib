double t2 = std::cos(q1);
double t3 = std::cos(q2);
double t4 = std::cos(q3);
double t5 = std::sin(q1);
double t6 = std::sin(q2);
double t7 = std::sin(q3);
double t8 = std::sqrt(3.0);
double t9 = r_b / 2.0;
double t10 = r_p / 2.0;
double t11 = -t10;
double t12 = t8 * t9;
double t13 = t8 * t10;
A0(0, 0) = l_p * t2 * (z - l_p * t5) * 2.0 + l_p * t5 *
                                                 (r_b - r_p - x + l_p * t2) *
                                                 2.0;
A0(1, 1) = l_p * t6 * (t9 + t11 + x + (l_p * t3) / 2.0) +
           l_p * t3 * (z - l_p * t6) * 2.0 -
           l_p * t6 * t8 * (t13 + y - (r_b * t8) / 2.0 - (l_p * t3 * t8) / 2.0);
A0(2, 2) = l_p * t7 * (t9 + t11 + x + (l_p * t4) / 2.0) +
           l_p * t4 * (z - l_p * t7) * 2.0 +
           l_p * t7 * t8 * (t12 + y - (r_p * t8) / 2.0 + (l_p * t4 * t8) / 2.0);
