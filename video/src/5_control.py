from manim import (
    FadeIn,
    FadeOut,
    MathTex,
    Scene,
    TexTemplate,
    TransformMatchingTex,
)


class Control(Scene):
    def construct(self) -> None:
        equation_1 = MathTex(
            r"{{ \ddot{\theta} &= f(\theta, \dot{\theta}) + }} \frac{\cos \theta}{l (m_1 + m_2) - l m_2 \cos^2 \theta} {{ F }} \\ {{ \ddot{x} &= g(\theta, \dot{\theta}) + }} \frac{2}{2 m_1 + m_2 - m_2 \cos 2 \theta} {{ F }}"
        )
        self.play(FadeIn(equation_1))
        self.wait(10)

        equation_2 = MathTex(
            r"{{ \ddot{\theta} &= f(\theta, \dot{\theta}) + }} \frac{1}{l m_1} {{ F }} \\ {{ \ddot{x} &= g(\theta, \dot{\theta}) + }} \frac{1}{m_1} {{ F }}"
        )
        self.play(TransformMatchingTex(equation_1, equation_2))
        self.wait(8)

        tex_template = TexTemplate()
        tex_template.add_to_preamble(
            "\n".join(
                [
                    r"\renewcommand{\vec}[1]{\boldsymbol{\mathbf{#1}}}",
                    r"\newcommand{\dvec}[1]{\dot{\vec{#1}}}",
                ],
            )
        )
        equation_3 = MathTex(
            r"{{ \dvec{x} = \vec{A} \vec{x} + \vec{B} F }} = \begin{bmatrix} 0 & 1 & 0 & 0 \\ \frac{g (m_1 + m_2)}{l m_1} & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 \\ \frac{g m_2}{m_1} & 0 & 0 & 0 \end{bmatrix} \vec{x} + \begin{bmatrix} 0 \\ \frac{1}{l m_1} \\ 0 \\ \frac{1}{m_1} \end{bmatrix} F",
            tex_environment="gather*",
            tex_template=tex_template,
        )
        self.play(FadeOut(equation_2), FadeIn(equation_3))
        self.wait(3.5)

        linearised_equation_tex = r"{{ \dvec{x} = \vec{A} \vec{x} + \vec{B} F }}"
        equation_4 = MathTex(
            linearised_equation_tex,
            tex_environment="gather*",
            tex_template=tex_template,
        )
        self.play(TransformMatchingTex(equation_3, equation_4))
        self.wait(19)

        equation_5 = MathTex(
            linearised_equation_tex,
            r"\\ \text{rank}(\vec{C}) = \text{rank}(\begin{bmatrix} \vec{B} & \vec{A} \vec{B} & \vec{A}^2 \vec{B} & \cdots & \vec{A}^{n - 1} \vec{B} \end{bmatrix}) = n",
            tex_environment="gather*",
            tex_template=tex_template,
        )
        self.play(TransformMatchingTex(equation_4, equation_5))
        self.wait(12)

        self.play(TransformMatchingTex(equation_5, equation_4))
        self.wait(18)

        equation_6 = MathTex(
            linearised_equation_tex,
            r"\\ {{ F = }} {{ \vec{K} \vec{x} }}",
            tex_environment="gather*",
            tex_template=tex_template,
        )
        self.play(TransformMatchingTex(equation_4, equation_6))
        self.wait(3)

        equation_7 = MathTex(
            linearised_equation_tex,
            r"\\ {{ F = }} - {{ \vec{K} \vec{x} }}",
            tex_environment="gather*",
            tex_template=tex_template,
        )
        self.play(TransformMatchingTex(equation_6, equation_7))
        self.wait(4)

        self.play(TransformMatchingTex(equation_7, equation_4))

        equation_8 = MathTex(
            r"{{ \dvec{x} = \vec{A} \vec{x} }} + {{ \vec{B} }} F",
            tex_template=tex_template,
        )
        self.replace(equation_4, equation_8)

        equation_9 = MathTex(
            r"{{ \dvec{x} = \vec{A} \vec{x} }} - {{ \vec{B} }} \vec{K} \vec{x}",
            tex_template=tex_template,
        )
        self.play(TransformMatchingTex(equation_8, equation_9))
        self.wait(1)

        equation_10 = MathTex(
            r"{{ \dvec{x} = }} \vec{A} \vec{x} - \vec{B} \vec{K} \vec{x}",
            tex_template=tex_template,
        )
        self.replace(equation_9, equation_10)

        equation_11 = MathTex(
            r"{{ \dvec{x} = }} (\vec{A} - \vec{B} \vec{K}) \vec{x}",
            tex_template=tex_template,
        ).shift([0, -0.05, 0])
        self.play(TransformMatchingTex(equation_10, equation_11))

        self.wait(1)
