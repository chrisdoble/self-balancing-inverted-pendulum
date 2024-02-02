from manim import (
    Angle,
    ArrowTriangleFilledTip,
    Circle,
    DashedLine,
    DEFAULT_FONT_SIZE,
    DOWN,
    FadeIn,
    FadeOut,
    LEFT,
    Line,
    MathTex,
    Mobject,
    MovingCameraScene,
    Rectangle,
    there_and_back,
    PI,
    rate_functions,
    RED,
    RIGHT,
    Transform,
    TransformMatchingTex,
    UP,
    ValueTracker,
    VGroup,
    VMobject,
    WHITE,
)
from math import cos, sin
from scipy.spatial.transform import Rotation
import numpy as np

LARGE_FONT_SIZE = DEFAULT_FONT_SIZE * 2


class EquationsOfMotion(MovingCameraScene):
    def construct(self) -> None:
        self.wait(4)

        # Cart
        cart = VGroup(Rectangle(height=1, width=2), MathTex("m_1")).shift([0, -1.5, 0])

        # Axes lines
        cart_right = cart.get_edge_center(RIGHT)
        axes_buffer = 1.5
        axes_size = 0.5
        axis_lines_points = [
            np.add(cart_right, [axes_buffer, axes_size, 0]),
            np.add(cart_right, [axes_buffer, 0, 0]),
            np.add(cart_right, [axes_buffer + axes_size, 0, 0]),
        ]
        axis_lines = VMobject(stroke_color=WHITE)
        axis_lines.start_new_path(axis_lines_points[0])
        axis_lines.add_points_as_corners(axis_lines_points[1:])

        # x axis tip
        x_axis_tip = ArrowTriangleFilledTip(
            fill_color=WHITE, length=0.2, start_angle=0, width=0.2
        ).next_to(axis_lines_points[2], RIGHT, buff=0)

        # x axis label
        x_axis_label = MathTex("x").next_to(x_axis_tip, RIGHT, buff=0.1)

        # y axis tip
        y_axis_tip = ArrowTriangleFilledTip(
            fill_color=WHITE, length=0.2, start_angle=PI / 2, width=0.2
        ).next_to(axis_lines_points[0], UP, buff=0)

        # y axis label
        y_axis_label = MathTex("y").next_to(y_axis_tip, UP, buff=0.1)

        self.play(
            FadeIn(axis_lines, x_axis_tip, x_axis_label, y_axis_tip, y_axis_label)
        )
        self.wait(2)
        self.play(FadeIn(cart))
        self.wait(2)

        # Dashed line between the cart and its origin
        cart_origin = np.add(cart.get_edge_center(LEFT), [-2.5, 0, 0])

        def update_cart_x_horizontal_line(m: Mobject) -> None:
            m.put_start_and_end_on(cart_origin, cart.get_edge_center(LEFT))

        cart_x_horizontal_line = DashedLine()
        update_cart_x_horizontal_line(cart_x_horizontal_line)
        cart_x_horizontal_line.add_updater(update_cart_x_horizontal_line)

        # Label above the dashed line
        def update_cart_x_label(m: Mobject) -> None:
            m.move_to(np.add(cart_x_horizontal_line.get_center(), [0, 0.25, 0]))

        cart_x_label = MathTex("x")
        update_cart_x_label(cart_x_label)
        cart_x_label.add_updater(update_cart_x_label)

        # Marker at the cart's origin
        cart_x_vertical_line = Line(
            end=np.add(cart_origin, [0, -0.25, 0]),
            start=np.add(cart_origin, [0, 0.25, 0]),
        )

        self.play(FadeIn(cart_x_horizontal_line, cart_x_label, cart_x_vertical_line))
        self.wait(2)
        self.play(cart.animate(rate_func=there_and_back).shift([-0.5, 0, 0]))
        self.wait(4)

        # Pendulum properties
        pendulum_angle = ValueTracker(PI / 9)
        pendulum_length = 3

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

        # Pendulum mass
        def update_pendulum_mass(m: Mobject) -> None:
            m.move_to(
                np.add(
                    pendulum_line.get_end(),
                    pendulum_line.get_unit_vector()
                    * pendulum_mass.submobjects[0].radius,
                )
            )

        pendulum_mass = VGroup(Circle(0.4, color=WHITE), MathTex("m_2"))
        update_pendulum_mass(pendulum_mass)
        pendulum_mass.add_updater(update_pendulum_mass)

        self.play(FadeIn(pendulum_line, pendulum_length_label, pendulum_mass))
        self.wait(2)

        # Vertical line
        def update_vertical_line(m: Mobject) -> None:
            cart_top = cart.get_edge_center(UP)
            m.put_start_and_end_on(cart_top, np.add(cart_top, [0, pendulum_length, 0]))

        vertical_line = DashedLine()
        update_vertical_line(vertical_line)
        vertical_line.add_updater(update_vertical_line)

        # Pendulum angle arc
        def update_pendulum_angle_arc(m: Mobject) -> None:
            m.become(Angle(vertical_line, pendulum_line, pendulum_length / 2))

        pendulum_angle_arc = Angle(vertical_line, pendulum_line)
        update_pendulum_angle_arc(pendulum_angle_arc)

        # Pendulum angle label
        def update_pendulum_angle_label(m: Mobject) -> None:
            arc_center = pendulum_angle_arc.point_from_proportion(0.5)
            to_arc_center = np.subtract(arc_center, cart.get_edge_center(UP))
            to_arc_center /= np.linalg.norm(to_arc_center)
            m.move_to(np.add(arc_center, to_arc_center / 2))

        pendulum_angle_label = MathTex(r"\theta")
        update_pendulum_angle_label(pendulum_angle_label)
        pendulum_angle_label.add_updater(update_pendulum_angle_label)

        self.play(FadeIn(vertical_line, pendulum_angle_arc, pendulum_angle_label))
        pendulum_angle_arc.add_updater(update_pendulum_angle_arc)
        self.play(pendulum_angle.animate(rate_func=there_and_back).set_value(PI / 6))
        self.wait(3)

        self.play(
            self.camera.frame.animate.move_to([9, -4, 0]).set(
                height=self.camera.frame.height * 2
            ),
        )

        self.wait(3)

        # General Lagrangian formula
        general_lagrangian = MathTex(
            r"\mathcal{L} = T - U", font_size=LARGE_FONT_SIZE
        ).move_to([12, -4, 0])
        self.play(FadeIn(general_lagrangian))
        self.wait(2.5)
        self.play(FadeOut(general_lagrangian))
        self.wait(3)

        # Cart kinetic energy
        cart_kinetic_energy_1 = MathTex(
            r"{{T_\text{cart} = \frac{1}{2} m_1}} {{v}}^2", font_size=LARGE_FONT_SIZE
        ).move_to([12, -4, 0])
        self.play(FadeIn(cart_kinetic_energy_1))
        self.wait(8)

        cart_kinetic_energy_2 = (
            MathTex(
                r"{{T_\text{cart} = \frac{1}{2} m_1}} {{\dot{x}}}^2",
                font_size=LARGE_FONT_SIZE,
            )
            .move_to(cart_kinetic_energy_1.get_center())
            .set_color_by_tex("dot", RED)
        )
        self.play(TransformMatchingTex(cart_kinetic_energy_1, cart_kinetic_energy_2))
        self.wait(3)
        self.play(cart_kinetic_energy_2.animate.set_color_by_tex("dot", WHITE))
        self.wait(6)
        self.play(cart_kinetic_energy_2.animate.move_to([0, -4, 0]))
        self.wait(6)

        # Cart potential energy
        cart_potential_energy = MathTex(
            r"U_\text{cart} = 0", font_size=LARGE_FONT_SIZE
        ).move_to([12, -4, 0])
        self.play(FadeIn(cart_potential_energy))
        self.wait(3.5)
        self.play(
            cart_potential_energy.animate.move_to([0, -6, 0]).align_to(
                cart_kinetic_energy_2, LEFT
            )
        )
        self.wait(2.5)

        self.play(cart.animate(rate_func=there_and_back, run_time=2).shift([-1, 0, 0]))
        self.wait(1)
        self.play(
            pendulum_angle.animate(rate_func=there_and_back, run_time=2).set_value(
                PI / 4
            )
        )
        self.wait(2)

        # Pendulum coordinates
        pendulum_x_coordinate = MathTex(
            r"X = x - l \sin \theta", font_size=LARGE_FONT_SIZE
        )
        pendulum_y_coordinate = (
            MathTex(r"Y = l \cos \theta", font_size=LARGE_FONT_SIZE)
            .next_to(pendulum_x_coordinate, DOWN, buff=0.75)
            .align_to(pendulum_x_coordinate, LEFT)
        )
        pendulum_coordinates = VGroup(
            pendulum_x_coordinate, pendulum_y_coordinate
        ).move_to([12, -4, 0])
        self.play(FadeIn(pendulum_coordinates))
        self.wait(5)

        # Pendulum velocities
        pendulum_x_velocity = (
            MathTex(
                r"\dot{X} = \dot{x} - l \dot{\theta} \cos \theta",
                font_size=LARGE_FONT_SIZE,
            )
            .align_to(pendulum_x_coordinate, DOWN)
            .align_to(pendulum_x_coordinate, LEFT)
        )
        pendulum_y_velocity = (
            MathTex(r"\dot{Y} = -l \dot{\theta} \sin \theta", font_size=LARGE_FONT_SIZE)
            .align_to(pendulum_y_coordinate, DOWN)
            .align_to(pendulum_y_coordinate, LEFT)
        )
        pendulum_velocities = VGroup(pendulum_x_velocity, pendulum_y_velocity)
        self.play(FadeOut(pendulum_coordinates), FadeIn(pendulum_velocities))
        self.wait(5.5)

        # Pendulum velocity squared
        pendulum_velocity_squared = MathTex(
            r"V^2 = {{\dot{x}^2 - 2 l \dot{\theta} \dot{x} \cos \theta + l^2 \dot{\theta}^2}}",
            font_size=LARGE_FONT_SIZE,
        ).move_to([12, -4, 0])
        self.play(FadeOut(pendulum_velocities), FadeIn(pendulum_velocity_squared))
        self.wait(2.5)

        # Pendulum kinetic energy
        pendulum_kinetic_energy = MathTex(
            r"T_\text{pendulum} = \frac{1}{2} m_2 ({{\dot{x}^2 - 2 l \dot{\theta} \dot{x} \cos \theta + l^2 \dot{\theta}^2}})",
            font_size=LARGE_FONT_SIZE,
        ).move_to([13.3, -4, 0])
        self.play(
            TransformMatchingTex(pendulum_velocity_squared, pendulum_kinetic_energy)
        )
        self.wait(2)
        self.play(
            pendulum_kinetic_energy.animate.move_to([0, -7.9, 0]).align_to(
                cart_potential_energy, LEFT
            )
        )
        self.wait(4)
        self.play(
            pendulum_angle.animate(rate_func=there_and_back, run_time=2).set_value(
                PI / 4
            )
        )
        self.wait(4)
        self.play(
            pendulum_angle.animate(
                rate_func=rate_functions.ease_in_out_quad, run_time=2
            ).set_value(PI / 2)
        )
        self.wait(2)

        # Pendulum potential energy
        pendulum_potential_energy = MathTex(
            r"U_\text{pendulum} = m_2 g l \cos \theta", font_size=LARGE_FONT_SIZE
        ).move_to([12, -4, 0])
        self.play(
            FadeIn(pendulum_potential_energy),
            pendulum_angle.animate(
                rate_func=rate_functions.ease_in_out_quad, run_time=2
            ).set_value(PI / 9),
        )
        self.wait(2)
        self.play(
            pendulum_potential_energy.animate.move_to([0, -10, 0]).align_to(
                pendulum_kinetic_energy, LEFT
            )
        )
        self.wait(3)

        # Lagrangian
        lagrangian = MathTex(
            r"\mathcal{L} = \frac{1}{2} (m_1 + m_2) \dot{x}^2 + \frac{1}{2} m_2 (l^2 \dot{\theta}^2 - 2 l \dot{\theta} \dot{x} \cos \theta) - m_2 g l \cos \theta",
            font_size=LARGE_FONT_SIZE,
        ).move_to([9, -5, 0])
        self.play(
            FadeOut(
                cart_kinetic_energy_2,
                cart_potential_energy,
                pendulum_kinetic_energy,
                pendulum_potential_energy,
            ),
            FadeIn(lagrangian),
        )
        self.wait(3)

        # Euler-Lagrange
        euler_lagrange = MathTex(
            r"\frac{\partial \mathcal{L}}{\partial q} - \frac{d}{d t} \frac{\partial \mathcal{L}}{\partial \dot{q}} = 0",
            font_size=LARGE_FONT_SIZE,
        ).next_to(lagrangian, DOWN, buff=0.75)
        self.play(FadeIn(euler_lagrange))
        self.wait(3)

        # Apply Euler-Lagrange to theta
        euler_lagrange_theta = MathTex(
            r"{{l \ddot{\theta} - \ddot{x} \cos \theta - g \sin \theta = 0}}",
            font_size=LARGE_FONT_SIZE,
        ).move_to([9, -5, 0])
        self.play(FadeOut(lagrangian, euler_lagrange), FadeIn(euler_lagrange_theta))
        self.wait(2)

        # Apply Euler-Lagrange to x
        euler_lagrange_x = MathTex(
            r"(m_1 + m_2) \ddot{x} - m_2 l \ddot{\theta} \cos \theta + m_2 l \dot{\theta}^2 \sin \theta = 0",
            font_size=LARGE_FONT_SIZE,
        ).move_to([9, -6.5, 0])
        self.play(FadeIn(euler_lagrange_x))
        self.wait(5)

        # Equations of motion
        equations_of_motion = MathTex(
            r"\ddot{\theta} & = \frac{(m_1 + m_2) g \sin \theta - m_2 l \dot{\theta}^2 \cos \theta \sin \theta}{l (m_1 + m_2) - m_2 l \cos^2 \theta} \\ \ddot{x}      & = \frac{m_2 \sin 2 \theta - 2 m_2 l \dot{\theta}^2 \sin \theta}{2 m_1 + m_2 - m_2 \cos 2 \theta}",
            font_size=LARGE_FONT_SIZE,
        ).move_to([9, -6.5, 0])
        self.play(
            FadeOut(euler_lagrange_theta, euler_lagrange_x), FadeIn(equations_of_motion)
        )

        self.wait()
