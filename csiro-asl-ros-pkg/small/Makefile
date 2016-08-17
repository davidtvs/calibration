
BZR_DIR = src
BZR_URL = lp:small
BZR_REVISION=1.0

DEST_DIR := include/small

all: $(DEST_DIR)

include $(shell rospack find mk)/bzr_checkout.mk

clean:
	@rm -fr $(BZR_DIR) $(DEST_DIR)

$(DEST_DIR): download
	@mkdir -p $@
	@cp -u $(BZR_DIR)/*.hh $@
