.PHONY: all
all: type_check
	manim src/1_equations_of_motion.py EquationsOfMotion

.PHONY: type_check
type_check:
	mypy src

.PHONY: clean
clean:
	rm -fr media src/__pycache__
