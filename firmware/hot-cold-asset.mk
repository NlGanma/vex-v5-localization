# if "template" is in the make command, do not include static.lib files
STATIC_ASSET_FILES=$(filter-out $(wildcard static/*.gif),$(wildcard static/*))
ifneq (,$(findstring template,$(MAKECMDGOALS)))
ASSET_FILES=$(STATIC_ASSET_FILES)
else
ASSET_FILES=$(STATIC_ASSET_FILES) $(wildcard static.lib/*)
endif

TEMPLATE_FILES+=$(wildcard static/*) $(wildcard firmware/hot-cold-asset.mk)

ASSET_OBJ=$(addprefix $(BINDIR)/, $(addsuffix .o, $(ASSET_FILES)) )

GETALLOBJ=$(sort $(call ASMOBJ,$1) $(call COBJ,$1) $(call CXXOBJ,$1)) $(ASSET_OBJ)

.SECONDEXPANSION:
$(ASSET_OBJ): $$(patsubst bin/%,%,$$(basename $$@))
	$(VV)mkdir -p $(BINDIR)/static
	$(VV)mkdir -p $(BINDIR)/static.lib
	@echo "ASSET $@"
	$(VV)$(OBJCOPY) -I binary -O elf32-littlearm -B arm $^ $@
