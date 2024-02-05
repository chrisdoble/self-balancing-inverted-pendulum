from manim import (
    BLUE,
    Circle,
    CurvedArrow,
    FadeIn,
    FadeOut,
    MathTex,
    PI,
    Scene,
    SVGMobject,
    TransformMatchingTex,
    WHITE,
)
import math
import os


class MotorControl(Scene):
    def construct(self) -> None:
        circuit_diagram = SVGMobject(
            os.path.join(os.path.dirname(__file__), "circuit_diagram.svg"),
            fill_color=WHITE,
            height=8,
            stroke_color=WHITE,
            stroke_width=2,
            width=8,
        ).shift([0, 1, 0])
        self.play(FadeIn(circuit_diagram))
        self.wait(18)

        electrical_equation_1 = MathTex(
            r"{{ V_\text{in} - I R - \frac{d I}{d t} L - }} V_\text{motor} {{ = 0 }}"
        ).shift([0, -1.5, 0])
        self.play(FadeIn(electrical_equation_1))
        self.wait(6)

        electrical_equation_2 = MathTex(
            r"{{ V_\text{in} - I R - \frac{d I}{d t} L - }} a_1 \omega {{ = 0 }}"
        ).shift([0, -1.5, 0])
        self.play(TransformMatchingTex(electrical_equation_1, electrical_equation_2))
        self.wait(7)

        self.play(FadeOut(circuit_diagram, electrical_equation_2))

        circle_y = 1
        circle = Circle(1, color=WHITE).shift([0, circle_y, 0])
        self.play(FadeIn(circle))
        self.wait(2)

        arc_angle = PI / 4
        arc_radius = 1.5
        tau_m_arc = CurvedArrow(
            [
                arc_radius * math.cos(arc_angle / 2),
                circle_y - arc_radius * math.sin(arc_angle / 2),
                0,
            ],
            [
                arc_radius * math.cos(arc_angle / 2),
                circle_y + arc_radius * math.sin(arc_angle / 2),
                0,
            ],
            angle=arc_angle,
        )
        tau_m_label = MathTex(r"\tau_m").move_to([2.1, circle_y, 0])
        self.play(FadeIn(tau_m_arc, tau_m_label))
        self.wait(2)

        tau_r_arc = CurvedArrow(
            [
                -arc_radius * math.cos(arc_angle / 2),
                circle_y - arc_radius * math.sin(arc_angle / 2),
                0,
            ],
            [
                -arc_radius * math.cos(arc_angle / 2),
                circle_y + arc_radius * math.sin(arc_angle / 2),
                0,
            ],
            angle=-arc_angle,
        )
        tau_r_label = MathTex(r"\tau_r").move_to([-2.1, circle_y, 0])
        self.play(FadeIn(tau_r_arc, tau_r_label))
        self.wait(2)

        mechanical_equation_1 = MathTex(
            r"{{ \tau_m }} {{ - }} {{ \tau_r }} {{ = J \dot{\omega} }}"
        ).shift([0, -1, 0])
        self.play(FadeIn(mechanical_equation_1))
        self.wait(8)

        mechanical_equation_2 = MathTex(
            r"{{ a_2 I }} {{ - }} {{ \tau_r }} {{ = J \dot{\omega} }}"
        ).shift([0, -1, 0])
        self.play(TransformMatchingTex(mechanical_equation_1, mechanical_equation_2))
        self.wait(6)

        mechanical_equation_3 = MathTex(
            r"{{ a_2 I }} {{ - }} {{ a_3 \omega }} {{ = J \dot{\omega} }}"
        ).shift([0, -1, 0])
        self.play(TransformMatchingTex(mechanical_equation_2, mechanical_equation_3))
        self.wait(11)

        equation_of_motion_1 = MathTex(
            r"b_1 \ddot{\omega} + b_2 \dot{\omega} + b_3 \omega + V_\text{in} = 0"
        )
        self.play(
            FadeOut(
                circle,
                tau_m_arc,
                tau_m_label,
                tau_r_arc,
                tau_r_label,
                mechanical_equation_3,
            ),
            FadeIn(equation_of_motion_1),
        )

        equation_of_motion_2 = MathTex(
            r"{{ \omega }} {{ = }} {{ c_1 V_\text{in} + c_2 e^{c_3 t} }} + c_4 e^{c_5 t}"
        )
        self.play(FadeOut(equation_of_motion_1), FadeIn(equation_of_motion_2))

        equation_of_motion_3 = MathTex(
            r"{{ \omega }} {{ = }} {{ c_1 V_\text{in} + c_2 e^{c_3 t} }}"
        )
        self.play(TransformMatchingTex(equation_of_motion_2, equation_of_motion_3))

        equation_of_motion_4 = MathTex(
            r"{{ \omega = c_1 V_\text{in} }} + c_2 e^{c_3 t}"
        )
        self.replace(equation_of_motion_3, equation_of_motion_4)

        equation_of_motion_5 = MathTex(
            r"{{ \omega = c_1 V_\text{in} }} (1 - e^{c_3 t})"
        ).shift([0.1, -0.03, 0])
        self.play(TransformMatchingTex(equation_of_motion_4, equation_of_motion_5))

        equation_of_motion_6 = MathTex(
            r"\omega {{ = }} c_1 {{ V_\text{in} (1 - }} e^{c_3 t})"
        ).shift([0.1, -0.03, 0])
        self.replace(equation_of_motion_5, equation_of_motion_6)

        equation_of_motion_7 = MathTex(
            r"v {{ = }} d_1 {{ V_\text{in} (1 - }} e^{-d_2 t})"
        ).shift([0.29, -0.02, 0])
        self.play(TransformMatchingTex(equation_of_motion_6, equation_of_motion_7))

        time_dependent_acceleration = MathTex(r"a = d_1 d_2 V_\text{in} e^{-d_2 t}")
        self.play(FadeOut(equation_of_motion_7), FadeIn(time_dependent_acceleration))
        self.play(FadeOut(time_dependent_acceleration))

        time_independent_acceleration_1 = MathTex(
            r"b_1 \ddot{\omega} + {{ b_2 \dot{\omega} + b_3 \omega + V_\text{in} = 0 }}"
        )
        self.play(FadeIn(time_independent_acceleration_1))

        time_independent_acceleration_2 = MathTex(
            r"{{ b_2 \dot{\omega} + b_3 \omega + V_\text{in} = 0 }}"
        )
        self.play(
            TransformMatchingTex(
                time_independent_acceleration_1, time_independent_acceleration_2
            )
        )

        time_independent_acceleration_3 = MathTex(
            r"{{ b_2 }} \dot{\omega} {{ + b_3 }} \omega {{ + }} {{ V_\text{in} = 0 }}"
        )
        self.replace(time_independent_acceleration_2, time_independent_acceleration_3)

        time_independent_acceleration_4 = MathTex(
            r"{{ b_2 }} a {{ + b_3 }} v {{ + }} R {{ V_\text{in} = 0 }}"
        )
        self.play(
            TransformMatchingTex(
                time_independent_acceleration_3, time_independent_acceleration_4
            )
        )

        time_independent_acceleration_5 = MathTex(
            r"b_2 {{ a }} + b_3 {{ v }} + R {{ V_\text{in} }} {{ = }} 0"
        )
        self.replace(time_independent_acceleration_4, time_independent_acceleration_5)

        time_independent_acceleration_6 = MathTex(
            r"{{ a }} {{ = }} k_1 {{ v }} + k_2 {{ V_\text{in} }}"
        )
        self.play(
            TransformMatchingTex(
                time_independent_acceleration_5, time_independent_acceleration_6
            )
        )

        time_independent_acceleration_7 = MathTex(
            r"{{ a }} {{ = }} -d_2 {{ v }} + d_1 d_2 {{ V_\text{in} }}"
        )
        self.play(
            TransformMatchingTex(
                time_independent_acceleration_6, time_independent_acceleration_7
            )
        )
