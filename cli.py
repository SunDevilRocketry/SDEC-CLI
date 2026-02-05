import cmd

class Cli(cmd.Cmd):
    intro = "SDECv2 CLI"
    prompt = ">> "

    def __init__(self):
        super().__init__()

    def do_quit(self, line):
        """
        Usage:
            quit
        """
        return True
    
    def do_q(self, line):
        """
        Usage:
            q
        """

        return self.do_quit(line)
    
if __name__=="__main__":
    Cli().cmdloop()