# Property helpers

# Parameter priority groups
PRIO_RESOLUTION = 4
PRIO_FRAME_TIME = 3
PRIO_EXPOSURE   = 2
PRIO_IO         = 1

def camProperty(notify=False, save=False, derivedFrom=None, prio=0):
    """@camProperty: Like @property, but include metadata.
        
        The camera properties, themselves, have certain
        properties we care about. They are all gettable,
        which we specify with a getter using @property,
        and can be settable. However, they can also emit a
        notification signal when they change, which is not
        a decorated feature. Some properties are derived
        from other properties, and we don't want to save
        those to disk. This decorator notes these last two
        properties for use later.
        
        'notify': The property emits update events when it
                  is changed (default: False)
        'save':   The property can be saved to disk to
                  preserve the configuration of the camera
                  class (default: False)
        'derivedFrom': The name of the backing property to save.
                  Needed, because aliased properties need to
                  save their master's value, or one of the
                  values will get overwritten.
        'prio':   The sort order to apply when setting multiple
                  properties at once. High numbers should be
                  set first (default: 0)
        
        Examples:
            set:
                @camProperty(notify=True, save=True)
                def exposurePeriod(self):
                    return ...
            get:
                type(self.camera).exposurePeriod
                    .fget.isNotifiable
        """
    def camPropertyAnnotate(fn, *args, **kwargs):
        """Helper function for camProperty decorator."""
        setattr(fn, 'notifies', notify)
        setattr(fn, 'saveable', save)
        setattr(fn, 'derivedFrom', derivedFrom)
        setattr(fn, 'prio', prio)
        return property(fn, *args, **kwargs)

    return camPropertyAnnotate
