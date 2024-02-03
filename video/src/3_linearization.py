from manim import (
    Axes,
    BLUE,
    config,
    Dot,
    FadeIn,
    FadeOut,
    GOLD,
    GrowFromCenter,
    MathTex,
    Mobject,
    rate_functions,
    Scene,
    TexTemplate,
    TransformMatchingTex,
    ValueTracker,
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

        self.play(FadeOut(axes, axis_labels, plot, point, linearized_plot))

        # Real-valued equations of motion
        equations_of_motion_1 = MathTex(
            r"{{ \ddot{\theta} }} & = {{ \frac{(m_1 + m_2) g \sin \theta - m_2 l \dot{\theta}^2 \cos \theta \sin \theta}{l (m_1 + m_2) - m_2 l \cos^2 \theta} }} \\ {{ \ddot{x} }} & = {{ \frac{m_2 \sin 2 \theta - 2 m_2 l \dot{\theta}^2 \sin \theta}{2 m_1 + m_2 - m_2 \cos 2 \theta} }}"
        )
        self.play(FadeIn(equations_of_motion_1))

        # Vector-valued equations of motion
        tex_template = TexTemplate()
        tex_template.add_to_preamble(
            r"\renewcommand{\vec}[1]{\boldsymbol{\mathbf{#1}}}"
        )
        equations_of_motion_2 = MathTex(
            r"{{ \begin{bmatrix} \ddot{\theta} \\ \ddot{x} \end{bmatrix} = \vec{f}(\theta, \dot{\theta}) = \begin{bmatrix} \frac{(m_1 + m_2) g \sin \theta - m_2 l \dot{\theta}^2 \cos \theta \sin \theta}{l (m_1 + m_2) - m_2 l \cos^2 \theta} \\ \frac{m_2 \sin 2 \theta - 2 m_2 l \dot{\theta}^2 \sin \theta}{2 m_1 + m_2 - m_2 \cos 2 \theta} \end{bmatrix} }}",
            tex_template=tex_template,
        )
        [
            ddot_theta,
            part_1,
            part_2,
            part_3,
            ddot_x,
            part_5,
            part_6,
        ] = equations_of_motion_1.submobjects
        self.play(
            FadeOut(part_1, part_2, part_3, part_5, part_6),
            ddot_theta.animate.shift([-0.43, -0.385, 0]),
            ddot_x.animate.shift([-0.415, 0.48, 0]),
        )
        self.play(FadeIn(equations_of_motion_2), FadeOut(ddot_theta, ddot_x))

        # Introduce state vector x
        equations_of_motion_3 = MathTex(
            r"{{ \begin{bmatrix} \ddot{\theta} \\ \ddot{x} \end{bmatrix} = \vec{f}(\theta, \dot{\theta}) = \begin{bmatrix} \frac{(m_1 + m_2) g \sin \theta - m_2 l \dot{\theta}^2 \cos \theta \sin \theta}{l (m_1 + m_2) - m_2 l \cos^2 \theta} \\ \frac{m_2 \sin 2 \theta - 2 m_2 l \dot{\theta}^2 \sin \theta}{2 m_1 + m_2 - m_2 \cos 2 \theta} \end{bmatrix} }} \\ \vec{x} = \begin{bmatrix} \theta \\ \dot{\theta} \\ x \\ \dot{x} \end{bmatrix}",
            tex_environment="gather*",
            tex_template=tex_template,
        )
        self.play(TransformMatchingTex(equations_of_motion_2, equations_of_motion_3))

        # Regroup LaTeX to support next animation
        equations_of_motion_4 = MathTex(
            r"{{ \begin{bmatrix} \ddot{\theta} \\ \ddot{x} \end{bmatrix} = \vec{f}( }} \theta, \dot{\theta} {{ ) = \begin{bmatrix} \frac{(m_1 + m_2) g \sin \theta - m_2 l \dot{\theta}^2 \cos \theta \sin \theta}{l (m_1 + m_2) - m_2 l \cos^2 \theta} \\ \frac{m_2 \sin 2 \theta - 2 m_2 l \dot{\theta}^2 \sin \theta}{2 m_1 + m_2 - m_2 \cos 2 \theta} \end{bmatrix} }} \\ {{ \vec{x} = \begin{bmatrix} \theta \\ \dot{\theta} \\ x \\ \dot{x} \end{bmatrix} }}",
            tex_environment="gather*",
            tex_template=tex_template,
        )
        self.remove(equations_of_motion_3)
        self.add(equations_of_motion_4)

        # Equations of motion updated to take x
        equations_of_motion_5 = MathTex(
            r"\begin{bmatrix} \ddot{\theta} \\ \ddot{x} \end{bmatrix} = \vec{f}( }} \vec{x} {{ ) = \begin{bmatrix} \frac{(m_1 + m_2) g \sin \theta - m_2 l \dot{\theta}^2 \cos \theta \sin \theta}{l (m_1 + m_2) - m_2 l \cos^2 \theta} \\ \frac{m_2 \sin 2 \theta - 2 m_2 l \dot{\theta}^2 \sin \theta}{2 m_1 + m_2 - m_2 \cos 2 \theta} \end{bmatrix} }} \\ {{ \vec{x} = \begin{bmatrix} \theta \\ \dot{\theta} \\ x \\ \dot{x} \end{bmatrix} }}",
            tex_environment="gather*",
            tex_template=tex_template,
        )
        self.play(TransformMatchingTex(equations_of_motion_4, equations_of_motion_5))

        self.wait()
