#!/bin/sh

set -e
SRC="http://pciids.sourceforge.net/v2.2/pci.ids"
DEST=pci.ids
PCI_COMPRESSED_IDS=
GREP=grep

if [ -n "$PCI_COMPRESSED_IDS" ] ; then
	DECOMP="cat"
	SRC="$SRC.gz"
	GREP=zgrep
elif which bzip2 >/dev/null ; then
	DECOMP="bzip2 -d"
	SRC="$SRC.bz2"
elif which gzip >/dev/null ; then
	DECOMP="gzip -d"
	SRC="$SRC.gz"
else
	DECOMP="cat"
fi

if which curl >/dev/null ; then
	DL="curl -o $DEST.new $SRC"
elif which wget >/dev/null ; then
	DL="wget -O $DEST.new $SRC"
elif which lynx >/dev/null ; then
	DL="eval lynx -source $SRC >$DEST.new"
else
	echo >&2 "update-pciids: cannot find curl, wget or lynx"
	exit 1
fi

if ! $DL ; then
	echo >&2 "update-pciids: download failed"
	rm -f $DEST.new
	exit 1
fi

if ! $DECOMP <$DEST.new >$DEST.neww ; then
	echo >&2 "update-pciids: decompression failed, probably truncated file"
	exit 1
fi

if ! $GREP >/dev/null "^C " $DEST.neww ; then
	echo >&2 "update-pciids: missing class info, probably truncated file"
	exit 1
fi

if [ -f $DEST ] ; then
	mv $DEST $DEST.old
	# --reference is supported only by chmod from GNU file, so let's ignore any errors
	chmod -f --reference=$DEST.old $DEST.neww 2>/dev/null || true
fi
mv $DEST.neww $DEST
rm $DEST.new

echo "Done."
