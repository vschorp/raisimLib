double t2 = std::cos(q2);
double t3 = std::cos(q3);
double t4 = x * 2.0;
double t5 = y * 2.0;
double t6 = z * 2.0;
double t7 = -r_p;
double t8 = std::sqrt(3.0);
double t9 = r_b * t8;
double t10 = r_p * t8;
A0(0, 0) = r_b * -2.0 + r_p * 2.0 + t4 - l_p * std::cos(q1) * 2.0;
A0(0, 1) = t5;
A0(0, 2) = t6 - l_p * std::sin(q1) * 2.0;
A0(1, 0) = r_b + t4 + t7 + l_p * t2;
A0(1, 1) = t5 - t9 + t10 - l_p * t2 * t8;
A0(1, 2) = t6 - l_p * std::sin(q2) * 2.0;
A0(2, 0) = r_b + t4 + t7 + l_p * t3;
A0(2, 1) = t5 + t9 + t7 * t8 + l_p * t3 * t8;
A0(2, 2) = t6 - l_p * std::sin(q3) * 2.0;
