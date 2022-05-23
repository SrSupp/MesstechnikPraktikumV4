#Please finish the method "sum_frames", in which two frames are added together.

def sum_frames(Input_Frame_a, Input_Frame_b):
    ausgabe = []
    #Your code here:
    #----------------------------------------

    #----------------------------------------
    return (ausgabe)

pi = 3.14159265359
Frame_Start = [0.2,0.05,0.3,*(pi,-pi/2,0)]
Frame_Offset = [0,0.1,-0.15,*(0,0,0)]
Frame_Ziel = sum_frames(Frame_Start,Frame_Offset)
print(Frame_Ziel)