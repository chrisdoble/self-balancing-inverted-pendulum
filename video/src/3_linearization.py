from manim import (
    Axes,
    BLUE,
    DOWN,
    config,
    DEFAULT_FONT_SIZE,
    Dot,
    FadeIn,
    FadeOut,
    GOLD,
    GrowFromCenter,
    MathTex,
    Mobject,
    MobjectMatrix,
    ORIGIN,
    rate_functions,
    Scene,
    TexTemplate,
    TransformMatchingTex,
    ValueTracker,
    VGroup,
)
from typing import Any, Callable


class Linearization(Scene):
    def construct(self) -> None:
        # Axes
        axes = Axes(
            axis_config={"include_numbers": True},
            x_range=[-1.999, 2, 1],
            y_range=[-2, 2, 1],
        )
        axis_labels = axes.get_axis_labels()

        # Non-linearized plot
        plot_function: Callable[[float], float] = lambda x: x**3 - x
        plot = axes.plot(plot_function, color=BLUE, x_range=[0, 0, 1])
        plot_limit = 1.51
        plot_max_x = ValueTracker(-plot_limit)

        def update_plot(m: Mobject) -> None:
            plot.become(
                axes.plot(
                    plot_function,
                    color=BLUE,
                    x_range=[-plot_limit, plot_max_x.get_value(), 0.01],
                )
            )

        plot.add_updater(update_plot)
        self.add(axes, axis_labels, plot)
        self.play(plot_max_x.animate(run_time=2).set_value(plot_limit))
        plot.remove_updater(update_plot)
        self.wait(5.5)

        # Point
        point = Dot(axes.coords_to_point(1, plot_function(1)), color=GOLD)
        self.play(
            GrowFromCenter(point), rate_func=rate_functions.ease_out_back, run_time=0.5
        )
        self.wait(3)

        # Derivative
        linearized_plot_function: Callable[[float], float] = lambda x: 2 * (x - 1)
        linearized_plot = axes.plot(linearized_plot_function, x_range=[0, 0, 1])
        linearized_plot_min_x = ValueTracker(1)
        linearized_plot_max_x = ValueTracker(1)

        def update_linearized_plot(m: Mobject) -> None:
            linearized_plot.become(
                axes.plot(
                    linearized_plot_function,
                    color=GOLD,
                    x_range=[
                        linearized_plot_min_x.get_value(),
                        linearized_plot_max_x.get_value(),
                        0.01,
                    ],
                )
            )

        linearized_plot.add_updater(update_linearized_plot)
        self.add(linearized_plot)
        self.play(
            linearized_plot_min_x.animate.set_value(0.8),
            linearized_plot_max_x.animate.set_value(1.2),
        )
        self.wait(5.5)
        self.play(
            linearized_plot_min_x.animate.set_value(0),
            linearized_plot_max_x.animate.set_value(1.96),
        )
        linearized_plot.remove_updater(update_linearized_plot)
        self.wait(42)

        self.play(FadeOut(axes, axis_labels, plot, point, linearized_plot))

        # Real-valued equations of motion
        equations_of_motion_1 = MathTex(
            r"{{ \ddot{\theta} }} & = {{ \frac{(m_1 + m_2) g \sin \theta - m_2 l \dot{\theta}^2 \cos \theta \sin \theta}{l (m_1 + m_2) - m_2 l \cos^2 \theta} }} \\ {{ \ddot{x} }} & = {{ \frac{m_2 \sin 2 \theta - 2 m_2 l \dot{\theta}^2 \sin \theta}{2 m_1 + m_2 - m_2 \cos 2 \theta} }}"
        )
        self.play(FadeIn(equations_of_motion_1))
        self.wait(5)

        # Move parts so they align with the vector-valued equation
        [
            ddot_theta_1,
            part_1,
            ddot_theta_equation_1,
            part_3,
            ddot_x_1,
            part_5,
            ddot_x_equation_1,
        ] = equations_of_motion_1.submobjects
        self.play(
            FadeOut(part_1, part_3, part_5),
            ddot_theta_1.animate.shift([-0.52, -0.37, 0]),
            ddot_theta_equation_1.animate.scale(2 / 3).shift([1.15, -0.31, 0]),
            ddot_x_1.animate.shift([-0.505, 0.295, 0]),
            ddot_x_equation_1.animate.scale(2 / 3).shift([2.4, 0.32, 0]),
        )

        # Output matrix
        ddot_theta_2 = MathTex(r"\ddot{\theta}")
        ddot_x_2 = MathTex(r"\ddot{x}")
        output_matrix_1 = MobjectMatrix(
            [
                [ddot_theta_2],
                [ddot_x_2],
            ],
        ).shift([-4.68, 0, 0])

        # = f(\theta, \dot{\theta}) =
        tex_template = TexTemplate()
        tex_template.add_to_preamble(
            "\n".join(
                [
                    r"\renewcommand{\vec}[1]{\boldsymbol{\mathbf{#1}}}",
                    r"\newcommand{\dvec}[1]{\dot{\vec{#1}}}",
                ],
            )
        )
        function_equality_1 = MathTex(
            r"{{= \vec{f}( }} \theta, \dot{\theta} {{ ) = }}",
            tex_template=tex_template,
        ).shift([-2.77, 0.04, 0])

        # Input matrix
        font_size = DEFAULT_FONT_SIZE * (2 / 3)
        ddot_theta_equation_2 = MathTex(
            r"\frac{(m_1 + m_2) g \sin \theta - m_2 l \dot{\theta}^2 \cos \theta \sin \theta}{l (m_1 + m_2) - m_2 l \cos^2 \theta}",
            font_size=font_size,
            tex_template=tex_template,
        )
        ddot_x_equation_2 = MathTex(
            r"\frac{m_2 \sin 2 \theta - 2 m_2 l \dot{\theta}^2 \sin \theta}{2 m_1 + m_2 - m_2 \cos 2 \theta}",
            font_size=font_size,
            tex_template=tex_template,
        )
        input_matrix_1 = MobjectMatrix(
            [
                [ddot_theta_equation_2],
                [ddot_x_equation_2],
            ],
            element_alignment_corner=ORIGIN,
        ).shift([1.65, 0, 0])

        self.play(FadeIn(output_matrix_1, function_equality_1, input_matrix_1))
        self.play(
            FadeOut(ddot_theta_1, ddot_x_1, ddot_theta_equation_1, ddot_x_equation_1)
        )
        self.wait(13)

        # Update vector-valued function to take x
        function_equality_2 = MathTex(
            r"{{= \vec{f}( }} \vec{x} {{ ) = }}",
            tex_template=tex_template,
        ).shift([-2.77, -0.005, 0])
        self.play(
            output_matrix_1.animate.shift([0.21, 0, 0]),
            TransformMatchingTex(function_equality_1, function_equality_2),
            input_matrix_1.animate.shift([-0.21, 0, 0]),
        )
        self.wait(2.5)

        # Animate things to make room for the new rows
        self.play(
            ddot_x_2.animate.shift([0, -0.795, 0]),
            output_matrix_1.get_brackets().animate.stretch_to_fit_height(3.4),
            ddot_theta_equation_2.animate.shift([0, 0.1, 0]),
            ddot_x_equation_2.animate.shift([0, -0.7, 0]),
            input_matrix_1.get_brackets().animate.stretch_to_fit_height(3.57),
        )

        # Ouput matrix with \dot{\theta} and \dot{x}
        output_matrix_2 = MobjectMatrix(
            [
                [MathTex(r"\dot{\theta}")],
                [ddot_theta_2.copy()],
                [MathTex(r"\dot{x}")],
                [ddot_x_2.copy()],
            ],
        ).shift([-4.47, 0, 0])

        # Input matrix with \dot{\theta} and \dot{x}
        input_matrix_2 = MobjectMatrix(
            [
                [MathTex(r"\dot{\theta}")],
                [ddot_theta_equation_2.copy()],
                [MathTex(r"\dot{x}")],
                [ddot_x_equation_2.copy()],
            ],
            element_alignment_corner=ORIGIN,
        ).shift([1.44, 0, 0])

        self.play(FadeIn(output_matrix_2, input_matrix_2))
        self.play(FadeOut(output_matrix_1, input_matrix_1))
        self.wait(2)

        # Change output matrix to the derivative of x
        state_vector_derivative = MathTex(
            r"\dvec{x}", tex_template=tex_template
        ).move_to([-4.05, 0.04, 0])
        self.play(FadeIn(state_vector_derivative), FadeOut(output_matrix_2))
        self.wait(1)

        # Move the equation
        vector_equation_of_motion = VGroup(
            state_vector_derivative, function_equality_2, input_matrix_2
        )
        self.play(vector_equation_of_motion.animate.shift([0, 0.5, 0]))

        # Linearisation equation
        linearisation_equation_1 = MathTex(
            r"\vec{f}(\vec{x}) \approx",
            r"f(",
            r"\vec{p}",
            r") + ",
            r"D \vec{f}|",
            r"_{\vec{p}}",
            r"(\vec{x} -",
            r"\vec{p}",
            r")",
            tex_template=tex_template,
        ).move_to([-0.36, -2, 0])
        self.play(FadeIn(linearisation_equation_1))
        self.wait(13.5)

        # Linearisation equation at zero vector
        linearisation_equation_2 = MathTex(
            r"\vec{f}(\vec{x}) \approx",
            r"f(",
            r"\vec{0}",
            r") + ",
            r"D \vec{f}|",
            r"_{\vec{0}}",
            r"(\vec{x} -",
            r"\vec{0}",
            r")",
            tex_template=tex_template,
        ).move_to([-0.36, -2, 0])
        self.play(
            TransformMatchingTex(linearisation_equation_1, linearisation_equation_2)
        )
        self.wait(30)

        # Simplified linearisation equation
        linearisation_equation_3 = MathTex(
            r"\vec{f}(\vec{x}) \approx",
            r"D \vec{f}|",
            r"_{\vec{0}}",
            r"\vec{x}",
            tex_template=tex_template,
        ).move_to([-1.75, -2, 0])
        self.play(
            TransformMatchingTex(linearisation_equation_2, linearisation_equation_3)
        )
        self.wait(29)

        # Final linearised function
        final_linearised_function = MathTex(
            r"\dvec{x} = \vec{A} \vec{x} = \begin{bmatrix} 0 & 1 & 0 & 0 \\ \frac{g (m_1 + m_2)}{l m_1} & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 \\ \frac{g m_2}{m_1} & 0 & 0 & 0 \end{bmatrix} \vec{x}",
            tex_template=tex_template,
        )
        self.play(
            FadeOut(vector_equation_of_motion, linearisation_equation_3),
            FadeIn(final_linearised_function),
        )

        self.wait()
