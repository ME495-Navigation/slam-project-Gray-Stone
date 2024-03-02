<!-- Be careful to page breaking when generating to pdf, the ASCII art is sensitive. -->
# Line segment and Ray intersect math

*Math here is inspired by content at https://stackoverflow.com/a/4030884*

Say the ray is something start at pint $P$. We want to check its intersection with line segment from $P1$ to $P2$

## Define all the vector components

Here we define these vectors:

```

                            ray_direction              P2                     
                           ^                      ---->                       
                           |              -------/    >                       
                           |      -------/          -/ ^                      
                          -x-----/                 /   |                      
                  -------/ | intersect            /    |                      
    P1    -------/         |                    -/     |                      
     <---/                 |                   /       |                      
       <                   |                  /        |                      
     ^  \-                 |                -/ V_a2    |                      
     |    \                |               /           |                      
     |     \-              |              /            |                      
     |       \ V_a1        |            -/             |                      
     |        \-           |           /               |                      
     |          \          |         -/                |V_b2                  
 V_b1|           \         |        /                  |                      
     |            \-       |       /                   |                      
     |              \      |     -/                    |                      
     |               \-    |    /                      |                      
     |                 \   |   /                       |                      
     |   V_c1           \- | -/         V_c2           |        Normal to ray 
     <------------------- \|---------------------------->---------->          
                            P                                                 

```

$V_{a1}$ is vector P-P1, $V_{c1}$ is the projection of $V_{a1}$ onto ray's normal direction.
$$ V_{c\ proj} = V_{a1} \cdot \hat{P_{norm}} $$ 
$$ V_{b\ proj} = V_{a1} \cdot \hat{V_{ray}} $$
The same apply for the other sets of vectors with $P2$. 

note: $V_{ray}$ and $P_{norm}$ are both unit vector

**Both $V_{b}$ and $V_c$ vector itself is not needed since we do the math later with geometry. But their projected length is important. As the sign after dot project is necessary for next step's checking.**

## Check for intersection

To check for intersection, we compare the direction of $V_{c1}$ and $V_{c2}$ (or the sign of $V_a \cdot P_{norm}$, as dot product gives directional magnitude after projection). This check will also give intersect result if both points are behind the line. Thus we also need to check for that. Could use the same doc product trick ${V_a} \cdot \hat{V_{ray}}$, negative means it's behind the ray and should be skipped.

<div style="page-break-after: always;"></div>

# Finding intersection point

```
                                                  
                                     ---^         
                               -----/   |  b2     
                         -----/         |         
                    ----/            ---|         
              -----/|          -----/   |         
        -----/      |     ----/         |         
     --/            -----/              |         
     ^         ----/|                   |  b2-b1  
  b1 |   -----/     | x                 |         
     |--/           |                   |         
     <--------------|------------------->         
           c1                c2                   
                                                  
                                                  
     ^--\                                         
     |   -----\                                   
     |         -----\                             
     |              |-----\                       
     |              |      -----\                 
   b1|              |            -----\           
     |              |                  ---^       
     |              |                     |       
     |              |                     | b2    
     |       c1     |        c2           |       
     <--------------|--------------------->       
      --\           |                     |       
         -----\     | x                   |       
               -----\                     | b2-b1 
                     -----\               |       
                           -----\         |       
                                 -----\   |       
                                       ---|  
```

The trick here is to move the line segment P1-P2 down to form two triangles.
We could use similar triangles to find the segment $x$, which $x+b_1$ will be the length from P to intersecting point.

**Sign of these values are important. For $b_1$ and $b_2$, they need to keep their sign from projection. While for $c_1$ and $c_2$, they need to be the magnitude (abs value)**

$$ {{x}\over {b_2-b_1}} = {{c_1} \over {c_1+c_2}} $$
$$ x= { c_1 (b_2-b_1) \over (c_1+c_2) } $$
$$ intersect\ length = { c_1 (b_2-b_1) \over (c_1+c_2) } + b_1 $$

This math  will work in both case where $b_1>b_2$ or $b_1< b_2$. In the second case, $x$ will be negative and subtract from $b_1$ will effectively "extend" it. 

There is a third case not drawn, which is $b_1$ and $b_2$ having opposite direction. The same math hold, as long as $b$ is a signed value (negative when it's going in opposite direction of the ray).