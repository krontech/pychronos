import pychronos

class zebra(pychronos.fpgamap):
    def __init__(self):
        super().__init__(0x7800, 0x100)

    def __regprop(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        fset=lambda self, value: self.regWrite(offset, size, value),
                        doc = docstring)
    def __regprop_ro(offset, size, docstring):
        return property(fget=lambda self: self.regRead(offset, size),
                        doc = docstring)

    identifier = __regprop_ro(0x00, 4, 'ID to make sure the module is where we think it is')
    version    = __regprop_ro(0x04, 4, 'major version')
    subversion = __regprop_ro(0x08, 4, 'minor version')
    status     = __regprop_ro(0x0C, 4, 'status register')
    
    control    = __regprop(0x10, 4, 'control register')

    threshold  = __regprop(0x20, 1, 'brightness threshold')
    
