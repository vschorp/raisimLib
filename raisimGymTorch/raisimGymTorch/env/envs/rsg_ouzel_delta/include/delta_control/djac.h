double t2 = std::cos(q1);
double t3 = std::cos(q2);
double t4 = std::cos(q3);
double t5 = std::sin(q1);
double t6 = std::sin(q2);
double t7 = std::sin(q3);
double t8 = q1 + q2;
double t9 = q1 + q3;
double t10 = q2 + q3;
double t11 = l_p * l_p;
double t12 = l_p * l_p * l_p;
double t13 = qd1 * qd1;
double t14 = qd2 * qd2;
double t15 = qd3 * qd3;
double t16 = r_b * r_b;
double t17 = r_p * r_p;
double t18 = xd * xd;
double t19 = yd * yd;
double t20 = zd * zd;
double t21 = r_b * y * 3.0;
double t22 = r_p * y * 3.0;
double t23 = r_b * z * 3.0;
double t24 = r_p * z * 3.0;
double t25 = xd * yd * 2.0;
double t26 = xd * zd * 2.0;
double t34 = -q1;
double t35 = -q2;
double t36 = -q3;
double t49 = std::sqrt(3.0);
double t27 = std::cos(t8);
double t28 = std::cos(t9);
double t29 = std::cos(t10);
double t30 = q3 + t8;
double t31 = std::sin(t8);
double t32 = std::sin(t9);
double t33 = std::sin(t10);
double t37 = l_p * t3 * y;
double t38 = l_p * t4 * y;
double t39 = l_p * r_b * t5;
double t40 = l_p * r_p * t5;
double t42 = t18 * 2.0;
double t43 = t19 * 2.0;
double t44 = t20 * 2.0;
double t45 = -t22;
double t46 = -t24;
double t47 = -t25;
double t48 = -t26;
double t50 = l_p * t2 * y * 2.0;
double t51 = l_p * t2 * z * 2.0;
double t52 = l_p * t5 * x * 2.0;
double t53 = l_p * t5 * y * 2.0;
double t54 = l_p * t6 * y * 2.0;
double t55 = l_p * t7 * y * 2.0;
double t59 = l_p * qdd2 * t6 * x;
double t60 = l_p * qdd3 * t7 * x;
double t61 = l_p * qd2 * t6 * xd;
double t62 = l_p * qd3 * t7 * xd;
double t63 = l_p * qd1 * t6 * yd;
double t64 = l_p * qd2 * t6 * yd;
double t65 = l_p * qd3 * t6 * yd;
double t66 = l_p * qd1 * t7 * zd;
double t67 = l_p * qd2 * t7 * zd;
double t68 = l_p * qd3 * t7 * zd;
double t69 = q1 + t35;
double t70 = q1 + t36;
double t71 = q2 + t36;
double t75 = t8 + t36;
double t76 = t9 + t35;
double t77 = t10 + t34;
double t85 = l_p * qdd2 * t3 * z * 2.0;
double t86 = l_p * qdd3 * t4 * z * 2.0;
double t87 = l_p * qd1 * t2 * zd * 4.0;
double t88 = l_p * qd2 * t3 * zd * 4.0;
double t89 = l_p * qd3 * t4 * zd * 4.0;
double t91 = l_p * qdd2 * r_b * t6 * 2.0;
double t92 = l_p * qdd3 * r_b * t7 * 2.0;
double t94 = l_p * qdd2 * r_p * t6 * 2.0;
double t95 = l_p * qdd3 * r_p * t7 * 2.0;
double t97 = l_p * qd2 * t5 * xd * 2.0;
double t98 = l_p * qd1 * t5 * xd * 4.0;
double t99 = l_p * qd3 * t5 * xd * 2.0;
double t103 = l_p * qdd1 * t2 * z * -2.0;
double t115 = l_p * r_b * t6 * y * 1.2E+1;
double t116 = l_p * r_b * t7 * y * 1.2E+1;
double t117 = l_p * r_p * t6 * y * 1.2E+1;
double t118 = l_p * r_p * t7 * y * 1.2E+1;
double t119 = r_b * t49 * x;
double t120 = r_p * t49 * x;
double t121 = r_b * t49 * z;
double t122 = r_p * t49 * z;
double t123 = l_p * t3 * t14 * x;
double t124 = l_p * t4 * t15 * x;
double t125 = t16 * t49;
double t126 = t17 * t49;
double t129 = r_b * r_p * t49 * 2.0;
double t130 = l_p * r_b * t2 * t13 * 2.0;
double t131 = l_p * r_b * t3 * t14 * 2.0;
double t132 = l_p * r_b * t4 * t15 * 2.0;
double t133 = l_p * r_p * t2 * t13 * 2.0;
double t134 = l_p * r_p * t3 * t14 * 2.0;
double t135 = l_p * r_p * t4 * t15 * 2.0;
double t136 = l_p * t2 * t13 * x * 2.0;
double t137 = l_p * t5 * t13 * z * 2.0;
double t138 = l_p * t6 * t14 * z * 2.0;
double t139 = l_p * t7 * t15 * z * 2.0;
double t142 = l_p * r_b * t2 * t49;
double t143 = l_p * r_b * t3 * t49;
double t144 = l_p * r_b * t4 * t49;
double t145 = l_p * r_p * t2 * t49;
double t146 = l_p * r_p * t3 * t49;
double t147 = l_p * r_p * t4 * t49;
double t148 = l_p * t3 * t49 * x;
double t149 = l_p * t4 * t49 * x;
double t150 = l_p * t3 * t49 * z;
double t151 = l_p * t4 * t49 * z;
double t162 = l_p * qdd2 * t6 * t49 * y;
double t163 = l_p * qdd3 * t7 * t49 * y;
double t172 = l_p * qd3 * t7 * t49 * yd * 2.0;
double t193 = l_p * r_b * r_p * t6 * t49 * 8.0;
double t194 = l_p * r_b * r_p * t7 * t49 * 8.0;
double t41 = std::sin(t30);
double t56 = -t42;
double t57 = -t43;
double t58 = -t44;
double t72 = std::cos(t69);
double t73 = std::cos(t70);
double t74 = std::cos(t71);
double t78 = std::sin(t69);
double t79 = std::sin(t70);
double t80 = std::sin(t71);
double t81 = -t39;
double t82 = -t52;
double t83 = -t54;
double t84 = qdd1 * t51;
double t90 = qdd1 * t39 * 2.0;
double t93 = qdd1 * t40 * 2.0;
double t96 = qdd1 * t52;
double t100 = std::sin(t75);
double t101 = std::sin(t76);
double t102 = std::sin(t77);
double t104 = -t85;
double t105 = -t87;
double t106 = -t88;
double t108 = -t91;
double t109 = -t95;
double t110 = -t59;
double t111 = -t61;
double t112 = -t63;
double t113 = -t64;
double t114 = -t65;
double t127 = -t116;
double t128 = -t117;
double t140 = t11 * t31 * y * 6.0;
double t141 = t11 * t32 * y * 6.0;
double t152 = t39 * t49;
double t153 = t40 * t49;
double t154 = -t129;
double t155 = -t130;
double t156 = -t131;
double t157 = -t135;
double t158 = -t123;
double t159 = -t139;
double t160 = r_p * t121 * 2.4E+1;
double t164 = t125 * z * 1.2E+1;
double t165 = t126 * z * 1.2E+1;
double t166 = -t147;
double t167 = l_p * t6 * t119 * 4.0;
double t168 = l_p * t7 * t119 * 4.0;
double t169 = l_p * t6 * t120 * 4.0;
double t170 = l_p * t7 * t120 * 4.0;
double t171 = t49 * t64 * 2.0;
double t175 = t14 * t37 * t49;
double t176 = t15 * t38 * t49;
double t180 = l_p * t5 * t125 * 4.0;
double t181 = l_p * t6 * t125 * 4.0;
double t182 = l_p * t7 * t125 * 4.0;
double t183 = l_p * t5 * t126 * 4.0;
double t184 = l_p * t6 * t126 * 4.0;
double t185 = l_p * t7 * t126 * 4.0;
double t186 = l_p * t2 * t121 * 8.0;
double t187 = l_p * t3 * t121 * 8.0;
double t188 = l_p * t4 * t121 * 8.0;
double t189 = l_p * t2 * t122 * 8.0;
double t190 = l_p * t3 * t122 * 8.0;
double t191 = l_p * t4 * t122 * 8.0;
double t204 = -t193;
double t205 = -t194;
double t207 = t11 * t27 * t49 * z * 2.0;
double t208 = t11 * t28 * t49 * z * 2.0;
double t209 = t11 * t29 * t49 * z * 2.0;
double t211 = r_b * t11 * t31 * t49 * 4.0;
double t212 = r_b * t11 * t32 * t49 * 4.0;
double t213 = r_b * t11 * t33 * t49 * 4.0;
double t214 = r_p * t11 * t31 * t49 * 4.0;
double t215 = r_p * t11 * t32 * t49 * 4.0;
double t216 = r_p * t11 * t33 * t49 * 4.0;
double t217 = t11 * t31 * t49 * x * 2.0;
double t218 = t11 * t32 * t49 * x * 2.0;
double t219 = t11 * t33 * t49 * x * 4.0;
double t107 = -t90;
double t161 = -t141;
double t173 = -t164;
double t174 = -t165;
double t177 = t11 * t78 * y * 2.0;
double t178 = t11 * t79 * y * 2.0;
double t179 = t11 * t80 * y * 4.0;
double t192 = r_p * t152 * 8.0;
double t195 = -t167;
double t196 = -t168;
double t197 = t152 * x * 8.0;
double t198 = t153 * x * 8.0;
double t200 = -t186;
double t201 = -t187;
double t202 = -t188;
double t210 = t12 * t41 * t49 * 3.0;
double t220 = -t207;
double t221 = -t208;
double t222 = -t209;
double t223 = -t214;
double t224 = -t215;
double t225 = -t216;
double t226 = -t219;
double t227 = t12 * t49 * t100;
double t228 = t12 * t49 * t101;
double t229 = t12 * t49 * t102;
double t230 = t11 * t49 * t72 * z * 2.0;
double t231 = t11 * t49 * t73 * z * 2.0;
double t232 = t11 * t49 * t74 * z * 2.0;
double t233 = t11 * t49 * t78 * x * 2.0;
double t234 = t11 * t49 * t79 * x * 2.0;
double t239 = t47 + t48 + t56 + t57 + t58 + t60 + t62 + t66 + t67 + t68 + t86 +
              t89 + t92 + t109 + t124 + t132 + t157 + t159 + t163 + t172 + t176;
double t240 = t25 + t26 + t42 + t43 + t44 + t94 + t104 + t106 + t108 + t110 +
              t111 + t112 + t113 + t114 + t134 + t138 + t156 + t158 + t162 +
              t171 + t175;
double t199 = -t177;
double t203 = -t192;
double t206 = -t198;
double t235 = -t230;
double t236 = -t231;
double t237 = -t232;
double t238 = t25 + t26 + t42 + t43 + t44 + t93 + t96 + t97 + t98 + t99 + t103 +
              t105 + t107 + t133 + t136 + t137 + t155;
double t241 = t115 + t118 + t127 + t128 + t140 + t160 + t161 + t169 + t170 +
              t173 + t174 + t178 + t179 + t180 + t181 + t182 + t183 + t184 +
              t185 + t189 + t190 + t191 + t195 + t196 + t197 + t199 + t200 +
              t201 + t202 + t203 + t204 + t205 + t206 + t210 + t211 + t212 +
              t213 + t217 + t218 + t220 + t221 + t222 + t223 + t224 + t225 +
              t226 + t227 + t228 + t229 + t233 + t234 + t235 + t236 + t237;
double t242 = 1.0 / t241;
A0(0, 0) = t239 * t242 *
               (t53 + t83 + t121 - t122 + t150 + t153 + t49 * t81 -
                t3 * t5 * t11 * t49) *
               -2.0 -
           t238 * t242 *
               (t55 + t83 + t121 * 2.0 - t122 * 2.0 + t150 + t151 -
                t11 * t33 * t49 - l_p * r_b * t6 * t49 - l_p * r_b * t7 * t49 +
                l_p * r_p * t6 * t49 + l_p * r_p * t7 * t49) *
               2.0 -
           t240 * t242 *
               (t53 - t55 - t121 + t122 - t151 + t152 - t153 +
                t4 * t5 * t11 * t49) *
               2.0;
A0(1, 0) = t239 * t242 *
               (t23 + t40 + t46 + t51 + t81 + t82 - l_p * r_b * t6 * 2.0 +
                l_p * r_p * t6 * 2.0 + l_p * t6 * x * 2.0 + l_p * t3 * z -
                t2 * t6 * t11 * 2.0 - t3 * t5 * t11) *
               -2.0 -
           t240 * t242 *
               (t23 + t40 + t46 + t51 + t81 + t82 - l_p * r_b * t7 * 2.0 +
                l_p * r_p * t7 * 2.0 + l_p * t7 * x * 2.0 + l_p * t4 * z -
                t2 * t7 * t11 * 2.0 - t4 * t5 * t11) *
               2.0 -
           l_p * t238 * t242 *
               (l_p * t80 + r_b * t6 - r_b * t7 - r_p * t6 + r_p * t7 +
                t6 * x * 2.0 - t7 * x * 2.0 + t3 * z - t4 * z) *
               2.0;
A0(2, 0) = t239 * t242 *
               (t21 + t37 + t45 + t50 + t119 - t120 - t125 - t126 + t129 -
                t142 - t143 + t145 + t146 + t148 - t2 * t3 * t11 * t49) *
               2.0 +
           t238 * t242 *
               (t37 - t38 + t119 * 2.0 - t120 * 2.0 + t125 + t126 + t143 +
                t144 - t146 + t148 + t149 + t154 + t166 + t3 * t4 * t11 * t49) *
               2.0 +
           t240 * t242 *
               (t21 + t38 + t45 + t50 - t119 + t120 + t125 + t126 + t142 +
                t144 - t145 - t149 + t154 + t166 + t2 * t4 * t11 * t49) *
               2.0;
