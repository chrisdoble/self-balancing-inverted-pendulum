.PHONY: all
all: type_check
	manim --preview --verbosity WARNING src/1_equations_of_motion.py EquationsOfMotion

.PHONY: type_check
type_check:
	mypy --no-error-summary src

.PHONY: clean
clean:
	rm -fr media src/__pycache__
