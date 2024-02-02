.PHONY: all
all: type_check
	manim ${MANIM_ARGS} --verbosity WARNING src/1_equations_of_motion.py EquationsOfMotion

.PHONY: clean
clean:
	rm -fr media src/__pycache__

.PHONY: preview
preview: MANIM_ARGS += --preview
preview: all

.PHONY: preview_last_frame
preview_last_frame: MANIM_ARGS += --save_last_frame
preview_last_frame: preview

.PHONY: type_check
type_check:
	mypy --no-error-summary src