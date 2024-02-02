MANIM_ARGS = --verbosity WARNING

.PHONY: all
all: type_check
	find src -name *.py -exec bash -c 'echo {} $$(grep -E "^class" {} | sed -Ee "s/^class ([^(]+).*/\1/")' \; \
		| xargs -L 1 manim ${MANIM_ARGS}

.PHONY: clean
clean:
	rm -fr media src/__pycache__

.PHONY: preview
preview: MANIM_ARGS += --preview
preview: render

.PHONY: preview_last_frame
preview_last_frame: MANIM_ARGS += --save_last_frame
preview_last_frame: preview

.PHONY: render
render: type_check
	@if [ -z ${FILE_NAME} ]; then echo "FILE_NAME must be defined" >&2; exit 1; fi
	manim ${MANIM_ARGS} ${FILE_NAME} $$(grep -E '^class' ${FILE_NAME} | sed -Ee 's/^class ([^()]+).*/\1/')

.PHONY: type_check
type_check:
	mypy --no-error-summary src