from svgpathtools import svg2paths
import re
l = []

def makePointFile(name):
    #Method 2
#    name = "curvetest5.svg"                          #PGedits
    paths, attributes = svg2paths(name)
    for k,v in enumerate(attributes):
        if(k == 0):
            s = v['d']
       # print(v['d'])   #Print out the values

    #Get the numbers into a list
    strang = str(s)
    for t in strang.split():
        num = re.findall(r"[-+]?\d*\.\d+|\d+", t)
       # print("Num:")
       # print(num)
        #print("***************")
        for x in num:
            try:
                l.append(float(x))
            except ValueError:
                pass
 #   print(l)

    f = open('testOut.txt', 'w')
    f.write("< d=")
    for y in l:
        f.write(str(y))
        f.write(",")
    f.write("/>")
    f.close()

    #return l  # Return the list of points

    #File Writing
    x = 0
    f = open('testOut.txt', 'w')
    f.write("< d=")
    wSpace = str(s)
    #print("****")
    #print(wSpace)
    #print("*****")
    noSpace = wSpace.replace(" ", "s")
    noSpace = noSpace.replace("L", "s")
    noSpace = noSpace.replace("Ms", "M")
    noSpace = noSpace.replace("ms", "m")
    noSpace = noSpace.replace("scs", "c")
    noSpace = noSpace.replace("sss", "s")
    noSpace = noSpace.replace("css", "c")
    noSpace = noSpace.replace("ssc", "c")
    noSpace = noSpace.replace(",-", "-")
    f.write(noSpace)
  #  print(noSpace)
    f.write("/>")
    f.close()



#print("pointlist prepared")
