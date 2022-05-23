double t2 = std::cos(q1);
double t3 = std::cos(q2);
double t4 = std::cos(q3);
double t5 = std::sin(q1);
double t6 = std::sin(q2);
double t7 = std::sin(q3);
double t8 = r_b * xdd;
double t9 = r_p * xdd;
double t10 = qd1 * qd1;
double t11 = qd2 * qd2;
double t12 = qd3 * qd3;
double t13 = xd * xd;
double t14 = yd * yd;
double t15 = zd * zd;
double t16 = x * xdd * 2.0;
double t17 = xd * yd * 2.0;
double t18 = y * ydd * 2.0;
double t19 = xd * zd * 2.0;
double t20 = z * zdd * 2.0;
double t21 = 1.0 / l_p;
double t26 = std::sqrt(3.0);
double t22 = t13 * 2.0;
double t23 = t14 * 2.0;
double t24 = t15 * 2.0;
double t25 = -t9;
double t27 = r_b * t26 * ydd;
double t28 = r_p * t26 * ydd;
A0(0, 0) = (t21 * (-t8 + t9 + t13 + t14 + t15 + x * xdd + xd * yd + y * ydd +
                   xd * zd + z * zdd - l_p * t2 * xdd - l_p * t5 * zdd -
                   l_p * r_b * t2 * t10 + l_p * r_p * t2 * t10 +
                   l_p * qd1 * t5 * xd * 2.0 + l_p * qd2 * t5 * xd +
                   l_p * qd3 * t5 * xd - l_p * qd1 * t2 * zd * 2.0 +
                   l_p * t2 * t10 * x + l_p * t5 * t10 * z)) /
           (r_b * t5 - r_p * t5 - t5 * x + t2 * z);
A0(1, 0) =
    (t21 *
     (t8 + t16 + t17 + t18 + t19 + t20 + t22 + t23 + t24 + t25 - t27 + t28 +
      l_p * t3 * xdd - l_p * t6 * zdd * 2.0 - l_p * r_b * t3 * t11 * 2.0 +
      l_p * r_p * t3 * t11 * 2.0 - l_p * qd2 * t6 * xd - l_p * qd1 * t6 * yd -
      l_p * qd2 * t6 * yd - l_p * qd3 * t6 * yd - l_p * qd2 * t3 * zd * 4.0 -
      l_p * t3 * t11 * x - l_p * t3 * t26 * ydd + l_p * t6 * t11 * z * 2.0 +
      l_p * qd2 * t6 * t26 * yd * 2.0 + l_p * t3 * t11 * t26 * y)) /
    (r_b * t6 * 2.0 - r_p * t6 * 2.0 + t6 * x + t3 * z * 2.0 - t6 * t26 * y);
A0(2,
   0) = (t21 *
         (t8 + t16 + t17 + t18 + t19 + t20 + t22 + t23 + t24 + t25 + t27 - t28 +
          l_p * t4 * xdd - l_p * t7 * zdd * 2.0 - l_p * r_b * t4 * t12 * 2.0 +
          l_p * r_p * t4 * t12 * 2.0 - l_p * qd3 * t7 * xd -
          l_p * qd3 * t4 * zd * 4.0 - l_p * qd1 * t7 * zd -
          l_p * qd2 * t7 * zd - l_p * qd3 * t7 * zd - l_p * t4 * t12 * x +
          l_p * t4 * t26 * ydd + l_p * t7 * t12 * z * 2.0 -
          l_p * qd3 * t7 * t26 * yd * 2.0 - l_p * t4 * t12 * t26 * y)) /
        (r_b * t7 * 2.0 - r_p * t7 * 2.0 + t7 * x + t4 * z * 2.0 +
         t7 * t26 * y);
