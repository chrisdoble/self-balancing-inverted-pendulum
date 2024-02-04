from manim import MathTex, Scene, TexTemplate, TransformMatchingTex


class Stability(Scene):
    def construct(self) -> None:
        tex_template = TexTemplate()
        tex_template.add_to_preamble(
            "\n".join(
                [
                    r"\renewcommand{\vec}[1]{\boldsymbol{\mathbf{#1}}}",
                    r"\newcommand{\dvec}[1]{\dot{\vec{#1}}}",
                ],
            )
        )
        equation_1 = MathTex(
            r"\dvec{x} = \vec{A} \vec{x} = \begin{bmatrix} 0 & 1 & 0 & 0 \\ \frac{g (m_1 + m_2)}{l m_1} & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 \\ \frac{g m_2}{m_1} & 0 & 0 & 0 \end{bmatrix} \vec{x}",
            tex_environment="gather*",
            tex_template=tex_template,
        )
        self.add(equation_1)

        equation_2 = MathTex(
            r"\dvec{x} = \vec{A} \vec{x} = \begin{bmatrix} 0 & 1 & 0 & 0 \\ \frac{g (m_1 + m_2)}{l m_1} & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 \\ \frac{g m_2}{m_1} & 0 & 0 & 0 \end{bmatrix} \vec{x}",
            r"\\ \\ 0, \ 0, \ -\sqrt{\frac{g (m_1 + m_2)}{l m_1}}, \ \sqrt{\frac{g (m_1 + m_2)}{l m_1}}",
            tex_environment="gather*",
            tex_template=tex_template,
        )
        self.play(TransformMatchingTex(equation_1, equation_2))
