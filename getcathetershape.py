# getcathetershape.py
# July 1, 2016, by Rachel Sassella
# for Armour Technologies.
# svg_to_points takes a .svg file and extracts and formats the path data from that file.
# In a regular .svg file, many of the points are relative to one another, and are connected
# by bezier curves. This function strips the data down to x,y pairs that trace out the shape
# of the catheter.
import matplotlib.pyplot as plt
import pointFile as pf 

def svg_to_points(filename):
    pf.makePointFile(filename)
    f = open("testOut.txt",'r')
    text = f.read()
    start = text.find(" d=")  #this marks where the coordinates begin
    start = start + 5   #get rid of " d=M"
    end = text.find("/>")
    text = text[start:end:1]        #this chops off everything before " d=" and after "/>"
    text = text.replace("s", "l")   #turns arcs and cubics into lines
    text = text.replace("c", "l")   #turns curvoto into lines
    text = text.replace("q", "l")   #turns quadratic bezior curves into lines
    text = text.replace("t", "l")   #turns smooth quadratic bezior curves into lines
    text = text.replace("a", "l")   #turns eliptical arc into lines
    text = text.replace("S", "L")   
    text = text.replace("C", "L")
    text = text.replace("T", "L")
    text = text.replace("Q", "L")
    text = text.replace("A", "L")
    text = text.replace('\n', '')   #gets rid of pesky newline and tab characters
    text = text.replace('\r', '')
    text = text.replace('\t', '')
    text = text.replace('\"', '')
    text = text.replace("l", ",l")  #organizes everything into nice csv format
    text = text.replace("L", ",L")  #Lineto
    text = text.replace("h", ",h")  #Horizontal lineto
    text = text.replace("H", ",H")  #Horizontal lineto
    text = text.replace("z", ",z")  #close shape
    text = text.replace("v", ",v")  #Vertical lineto
    text = text.replace("V", ",V")  #Vertical lineto
    text = text.replace("-", ",-")  
    text = text.replace("l,", "l")
    text = text.replace("L,", "L")
    text = text.replace("h,", "h")
    text = text.replace("H,", "H")
    text = text.replace("z,", "z")
    text = text.replace("v,", "v")
    text = text.replace("V,", "V")
    pointlist = text.split(",")     # csv data gets fed in
    #print(pointlist)
    flag = False
    tab = False                #
    positions = []
    n = 0
    #this for loop turns all relative data into absolute data.
    for point in pointlist:
        index = n
    
#        print(index)
    
    #the first "l"<lowercase> marks the point at which data becomes relative.
    # a <uppercase> "L" will signify absolute data once again
        if point[0] == "l": # l as in lemon
            flag = True
        if point[0] == "L":
            flag = False
        if (point[0] == "h") or (point[0] == "v"):
            flag = True
            tab = True
        if (point[0] == "H") or (point[0] == "V"):
            flag = False
            tab = True
        if (point[0] == "z"):
            point = point.replace("z", "")
            break
        #print("flag ", flag)
        #print ("tab ",  tab)
        
        # add each x and y to previous x and y to get absolute positions.
        # store those positions in positions.
        if (flag):
            last = positions[index-2]
            this = list(filter(lambda x: x not in "l", point))
            #print("this:")
            this = list(filter(lambda x: x not in "h", this))
            this = list(filter(lambda x: x not in "v", this))
           # print("this:")
           # print(this)
            num = "".join(this)  # This line is super important. It makes all the numbers in the list turn into something usable.
           # print("num:")
           # print(num)
           # print(last)
            y = float(num) + float(last)
           # print(y)
            positions.append(y)
        if (tab):
            lasty = positions[index-1]
            positions.append(lasty)
            tab = False
        if (not flag):
            this = list(filter(lambda x: x not in "L", point))
            this = list(filter(lambda x: x not in "H", this))
            this = list(filter(lambda x: x not in "h", this))
            this = list(filter(lambda x: x not in "V", this))
           # print("this not:")
           # print(this)
            num = "".join(this) #This line is super important. It makes all the numbers in the list turn into something usable.
           # print(num)
            num = float(num)/2.83
            positions.append(num)  #turn strings into floats
#        print("positions:")
#        print(positions)
        n = n + 1


    positions = list(filter(None, positions))
    x = positions[0::2]
    # the y data is inverted---it's pertier if you flip it back.
    yold = positions[1::2]
    y = []
    for i in yold:
        y.append(-i)

    #uncomment if you want to look at the nice picture.
#    chop = len(x)/2 + 3
#    x = x[:chop]
#    y = y[:chop]
#    plt.plot(x, y, 'ro')
#    plt.axis('equal')
#    plt.show()

    f.close()
    #print(len(x))
    #print("x")
    #print x
    #print(len(y))
    #print("y")
    #print y
    return x, y

#l = svg_to_points("testOut.txt") #make this "curvetest.svg" to work

#print(l)