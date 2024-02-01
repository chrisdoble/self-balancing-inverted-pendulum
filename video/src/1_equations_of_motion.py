from manim import (
    Angle,
    Circle,
    DashedLine,
    LEFT,
    Line,
    MathTex,
    Mobject,
    Rectangle,
    Scene,
    there_and_back,
    PI,
    UP,
    ValueTracker,
    VGroup,
    WHITE,
)
from math import cos, sin
from scipy.spatial.transform import Rotation
import numpy as np


class EquationsOfMotion(Scene):
    def construct(self) -> None:
        # Cart
        cart = VGroup(Rectangle(height=1, width=2), MathTex("m_1")).shift([0, -1, 0])
        self.add(cart)

        # Dashed line between the cart and its origin
        cart_origin = np.add(cart.get_edge_center(LEFT), [-2.5, 0, 0])

        def update_cart_x_horizontal_line(m: Mobject) -> None:
            m.put_start_and_end_on(cart_origin, cart.get_edge_center(LEFT))

        cart_x_horizontal_line = DashedLine()
        update_cart_x_horizontal_line(cart_x_horizontal_line)
        cart_x_horizontal_line.add_updater(update_cart_x_horizontal_line)
        self.add(cart_x_horizontal_line)

        # Label above the dashed line
        def update_cart_x_label(m: Mobject) -> None:
            m.move_to(np.add(cart_x_horizontal_line.get_center(), [0, 0.25, 0]))

        cart_x_label = MathTex("x")
        update_cart_x_label(cart_x_label)
        cart_x_label.add_updater(update_cart_x_label)
        self.add(cart_x_label)

        # Marker at the cart's origin
        cart_x_vertical_line = Line(
            start=np.add(cart_origin, [0, 0.25, 0]),
            end=np.add(cart_origin, [0, -0.25, 0]),
        )
        self.add(cart_x_vertical_line)

        # Pendulum properties
        pendulum_angle = ValueTracker(PI / 9)
        pendulum_length = 3
        pendulum_mass_radius = 0.4

        # Pendulum line
        def update_pendulum_line(m: Mobject) -> None:
            cart_top = cart.get_edge_center(UP)
            angle = pendulum_angle.get_value()
            m.put_start_and_end_on(
                cart_top,
                np.add(
                    cart_top,
                    pendulum_length * np.array([-sin(angle), cos(angle), 0]),
                ),
            )

        pendulum_line = Line()
        update_pendulum_line(pendulum_line)
        pendulum_line.add_updater(update_pendulum_line)
        self.add(pendulum_line)

        # Pendulum length label
        def update_pendulum_length_label(m: Mobject) -> None:
            m.move_to(
                np.add(
                    pendulum_line.get_center(),
                    Rotation.from_euler("z", PI / 2).apply(
                        pendulum_line.get_unit_vector()
                    )
                    / 2,
                )
            )

        pendulum_length_label = MathTex("l")
        update_pendulum_length_label(pendulum_length_label)
        pendulum_length_label.add_updater(update_pendulum_length_label)
        self.add(pendulum_length_label)

        # Pendulum mass
        def update_pendulum_mass(m: Mobject) -> None:
            m.move_to(
                np.add(
                    pendulum_line.get_end(),
                    pendulum_line.get_unit_vector() * pendulum_mass_radius,
                )
            )

        pendulum_mass = VGroup(
            Circle(pendulum_mass_radius, stroke_color=WHITE), MathTex("m_2")
        )
        update_pendulum_mass(pendulum_mass)
        pendulum_mass.add_updater(update_pendulum_mass)
        self.add(pendulum_mass)

        # Vertical line
        def update_vertical_line(m: Mobject) -> None:
            cart_top = cart.get_edge_center(UP)
            m.put_start_and_end_on(cart_top, np.add(cart_top, [0, pendulum_length, 0]))

        vertical_line = DashedLine()
        update_vertical_line(vertical_line)
        vertical_line.add_updater(update_vertical_line)
        self.add(vertical_line)

        # Pendulum angle arc
        def update_pendulum_angle_arc(m: Mobject) -> None:
            m.become(Angle(vertical_line, pendulum_line, pendulum_length / 2))

        pendulum_angle_arc = Angle(vertical_line, pendulum_line)
        update_pendulum_angle_arc(pendulum_angle_arc)
        pendulum_angle_arc.add_updater(update_pendulum_angle_arc)
        self.add(pendulum_angle_arc)

        # Pendulum angle label
        def update_pendulum_angle_label(m: Mobject) -> None:
            arc_center = pendulum_angle_arc.point_from_proportion(0.5)
            to_arc_center = np.subtract(arc_center, cart.get_edge_center(UP))
            to_arc_center /= np.linalg.norm(to_arc_center)
            m.move_to(np.add(arc_center, to_arc_center / 2))

        pendulum_angle_label = MathTex(r"\theta")
        update_pendulum_angle_label(pendulum_angle_label)
        pendulum_angle_label.add_updater(update_pendulum_angle_label)
        self.add(pendulum_angle_label)

        self.wait()
        self.play(cart.animate(rate_func=there_and_back).shift([-0.5, 0, 0]))
        self.play(pendulum_angle.animate(rate_func=there_and_back).set_value(PI / 6))
        self.wait()
