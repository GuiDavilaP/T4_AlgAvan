CXX = g++
CXXFLAGS = -O3 -I./src
OBJDIR = obj
SRCDIR = src
INCDIR = inc
BINDIR = bin

# Lista de arquivos objeto para o emparelhamento
OBJECTS = $(OBJDIR)/algoritmoHungaro.o $(OBJDIR)/readGraph.o

# Lista de arquivos objeto para o gerador
GENERATOR_OBJECTS = $(OBJDIR)/graphGenerator.o

# Regras principais
all: $(BINDIR)/emparelhamento $(BINDIR)/generator

$(BINDIR)/emparelhamento: $(OBJECTS)
	@mkdir -p $(BINDIR)
	$(CXX) $(OBJECTS) -o $@

$(BINDIR)/generator: $(GENERATOR_OBJECTS)
	@mkdir -p $(BINDIR)
	$(CXX) $(GENERATOR_OBJECTS) -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Python virtual environment and analysis
venv:
	python3 -m venv venv
	. venv/bin/activate && pip install -r requirements.txt

analyze: venv
	. venv/bin/activate && python3 analyseResults.py

clean-venv:
	rm -rf venv/
	rm -rf __pycache__/

# Update clean target
clean: clean-venv
	rm -rf $(OBJDIR)/* $(BINDIR)/*

.PHONY: venv analyze clean-venv