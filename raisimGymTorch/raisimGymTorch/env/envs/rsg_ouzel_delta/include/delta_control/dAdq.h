double t2 = std::sin(q2);
double t3 = std::sin(q3);
double t4 = std::sqrt(3.0);
A0(0, 0) = l_p * qd1 * std::sin(q1) * 2.0;
A0(0, 2) = l_p * qd1 * std::cos(q1) * -2.0;
A0(1, 0) = -l_p * qd2 * t2;
A0(1, 1) = l_p * qd2 * t2 * t4;
A0(1, 2) = l_p * qd2 * std::cos(q2) * -2.0;
A0(2, 0) = -l_p * qd3 * t3;
A0(2, 1) = -l_p * qd3 * t3 * t4;
A0(2, 2) = l_p * qd3 * std::cos(q3) * -2.0;
