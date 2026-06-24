VARIANTS :=
VARIANTS += example
VARIANTS += example_splitter_mode

.PHONY: all clean $(VARIANTS)

all: $(VARIANTS)

$(VARIANTS):
	$(MAKE) -C $@

clean:
	for d in $(VARIANTS); do $(MAKE) -C $$d clean; done
