CXX = g++
CXXFLAGS = -O3 -I./src
OBJDIR = obj
SRCDIR = src
INCDIR = inc
BINDIR = bin

# Lista de arquivos objeto
OBJECTS = $(OBJDIR)/algoritmoHungaro.o $(OBJDIR)/readGraph.o

# Regra principal
all: $(BINDIR)/emparelhamento

$(BINDIR)/emparelhamento: $(OBJECTS)
	@mkdir -p $(BINDIR)
	$(CXX) $(OBJECTS) -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJDIR)/* $(BINDIR)/*