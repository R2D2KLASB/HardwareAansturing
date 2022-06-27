## @package beeldverwerking.handler.eventhandler
#  Make and run events.


class eventHandler():
    ## constructor
    def __init__(self):
        ## An list with dicts with a name(string) and func(function)
        self.events = []

    ## Create an event with an given name and function in an list
    def createEvent(self, name, func):
        self.events+=[{
            'name': name,
            'func': func,
        }]

    ## Returns an function from an list with an given name
    def runEvent(self, name):
        for index in range(len(self.events)):
            if self.events[index]['name'] == name:
                return self.events[index]['func']
        print('EventHandler: Nothing found')
        return None