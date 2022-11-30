def parse_obj(path):
    with open(path, 'r') as file:
        raw = file.readlines()
    
    #parse verts
    verts = []
    faces =[]
    for line in raw:
        if line.split(' ')[0].lower() == 'v':
            verts.append(line.replace('v ', ''))
            
        
        if line.split(' ')[0].lower() == 'f':
            faces.append(line.replace('f ', ''))
            
    #print(verts[0][1])
    temp = []
    for vert in verts:
        x, y, z = vert.split(' ')

        x = float(x)
        y = float(y)
        z = float(z)


        temp.append([x, y, z])
    verts = temp
    #print(verts[0][2])

    temp = []
    for face in faces:
        face = face.split(' ')
        f = []
        for i in face:
            f.append(int(i.split('/')[0]))
        temp.append(f)
    faces = temp
    print(len(faces))
    print(type(faces[0][0]))
    print(faces[519])
    print(faces[0][0]-1)
    print(verts[1])
    print(verts[1][0])
    
    faces_in_xyz = [[[0 for _ in range(3)] for _ in range(3)] for _ in range(520)]
    
    
    print(faces_in_xyz)
    for i in range(0,520):
        face1_index = int(faces[i][0])-1
        face2_index = int(faces[i][1])-1
        face3_index = int(faces[i][2])-1
        face1_xvalue = verts[face1_index][0]
        face1_yvalue = verts[face1_index][1]
        face1_zvalue = verts[face1_index][2]
        face2_xvalue = verts[face2_index][0]
        face2_yvalue = verts[face2_index][1]
        face2_zvalue = verts[face2_index][2]
        face3_xvalue = verts[face3_index][0]
        face3_yvalue = verts[face3_index][1]
        face3_zvalue = verts[face3_index][2]
        
        faces_in_xyz[i][0][0] = face1_xvalue
        faces_in_xyz[i][0][1] = face1_yvalue
        faces_in_xyz[i][0][2] = face1_zvalue
        
        faces_in_xyz[i][1][0] = face2_xvalue
        faces_in_xyz[i][1][1] = face2_yvalue
        faces_in_xyz[i][1][2] = face2_zvalue
        
        faces_in_xyz[i][2][0] = face3_xvalue
        faces_in_xyz[i][2][1] = face3_yvalue
        faces_in_xyz[i][2][2] = face3_zvalue
    
    print(faces_in_xyz)
    

            
# parse_obj('data/cube.obj')            
parse_obj('data/cube_cover.obj')            

# import numpy as np
# f = open('map.txt', 'r')
# raw = f.readlines()
# raw = raw[0].split(']]')
# raw = raw[:-1]
# for index in range(len(raw)):
#     if index == 0:
#         raw[index] = raw[index][2:]
#     else:
#         raw[index] = raw[index][4:]
#     raw[index] = raw[index].split(',')

#     raw[index][1] = raw[index][1][1:]
#     raw[index][2] = raw[index][2][1:-1]
#     raw[index][3] = raw[index][3][2:]
#     raw[index][4] = raw[index][4][1:]
#     raw[index][5] = raw[index][5][1:-1]
#     raw[index][6] = raw[index][6][2:]
#     raw[index][7] = raw[index][7][1:]
#     raw[index][8] = raw[index][8][1:]

# for i in range(508):
#     p0 = np.array([float(raw[i][0]) / 90.0, float(raw[i][1]) / 90.0, float(raw[i][2]) / 90.0])
#     p1 = np.array([float(raw[i][3]) / 90.0, float(raw[i][4]) / 90.0, float(raw[i][5]) / 90.0])
#     p2 = np.array([float(raw[i][6]) / 90.0, float(raw[i][7]) / 90.0, float(raw[i][8]) / 90.0])