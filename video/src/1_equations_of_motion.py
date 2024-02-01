from manim import Circle, Create, PINK, Scene


class EquationsOfMotion(Scene):
    def construct(self) -> None:
        circle = Circle()
        circle.set_fill(PINK, opacity=0.5)
        self.play(Create(circle))
