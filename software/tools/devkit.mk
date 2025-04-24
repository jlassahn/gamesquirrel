
DEVKIT_FILES := \
docs/game_programmer_guide.pdf \
docs/hardware_reference.pdf \
docs/system_internals.pdf \
docs/user_guide.pdf \
README.txt


build/README.txt: ../docs/README.txt
	cp $< $@

build/devkit.tgz: $(addprefix build/, $(DEVKIT_FILES))
	tar -czf build/devkit.tgz --directory=build $(DEVKIT_FILES)

build/docs/%.pdf: ../docs/%.adoc
	asciidoctor-pdf -o $@ $<

