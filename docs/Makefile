# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SPHINXAPIDOC  ?= sphinx-apidoc
SOURCEDIR     = source
BUILDDIR      = build
PROJECTROOT   = ..
APIDOCOUT     = $(SOURCEDIR)/api/generated

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help apidoc html Makefile

apidoc:
	@mkdir -p "$(APIDOCOUT)"
	@$(SPHINXAPIDOC) -f -e -M -o "$(APIDOCOUT)" "$(PROJECTROOT)" \
		"$(PROJECTROOT)/.venv" \
		"$(PROJECTROOT)/docs" \
		"$(PROJECTROOT)/__pycache__" \
		"$(PROJECTROOT)/main.py"

html: apidoc
	@$(SPHINXBUILD) -M html "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
