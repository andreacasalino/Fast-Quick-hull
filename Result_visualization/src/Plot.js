let vertex_shader = document.createElement("script");
vertex_shader.setAttribute("id", "3d-vertex-shader");
vertex_shader.setAttribute("type", "x-shader/x-vertex");
vertex_shader.innerHTML = "attribute vec4 a_position;"+
"attribute vec4 a_color;"+
"uniform mat4 u_matrix;"+
"varying vec4 v_color;"+
"void main() {"+
"    gl_Position = u_matrix * a_position;"+
"    gl_Position.x = gl_Position.x / (gl_Position.z + 2.0);" +
"    gl_Position.y = gl_Position.y / (gl_Position.z + 2.0);" +
"    v_color = a_color;" +
"}";
document.body.appendChild(vertex_shader);

let fragment_shader = document.createElement("script");
fragment_shader.setAttribute("id", "3d-fragment-shader");
fragment_shader.setAttribute("type", "x-shader/x-fragment");
fragment_shader.innerHTML = "precision mediump float;"+
"varying vec4 v_color;"+
"void main() {"+
"    gl_FragColor = v_color;"+
"}";
document.body.appendChild(fragment_shader);


function radToDeg(r) {
    return r * 180 / Math.PI;
}

function degToRad(d) {
    return d * Math.PI / 180;
}

class WebGL_Plot{
    constructor(Canvas_sizes, H_window, D_window, background_color = [0,0,0]){

        this.canvas = document.createElement("canvas");
        this.gl = this.canvas.getContext("webgl");
        if (!this.gl) {
            alert("WebGL not supported");
            return;
        }
        this.bckgrn = background_color;
        this.canvas.width = Canvas_sizes[0];
        this.canvas.height = Canvas_sizes[1];
        this.Camera_width = H_window * Canvas_sizes[0] / Canvas_sizes[1];
        this.Camera_height = H_window;

        this.gl.viewport(0, 0, this.canvas.width, this.canvas.height);
        this.gl.enable(this.gl.CULL_FACE);
        this.gl.enable(this.gl.DEPTH_TEST);
        this.programs = [];
        this.id_vrtx_shader =  "3d-vertex-shader";
        this.id_frag_shader =  "3d-fragment-shader";
        
        this.mProjection = [[2.0/this.Camera_width, 0, 0, 0],
                            [0,2.0/this.Camera_height, 0, 0],
                            [0,0,2.0/D_window,0],
                            [0,0,-1,1]];

        this.move_camera([0,0], 0);
    };

    get_canvas(){ return this.canvas; };

    add_Mesh(Vertices, color){
        this.__add_vertices(Vertices, color, this.gl.TRIANGLES);
    };
    
    add_Segments(coordinates, color){
        this.__add_vertices(coordinates, color, this.gl.LINES);
    };
    
    add_Points(coordinates, color){
        this.__add_vertices(coordinates, color, this.gl.POINTS);
    };
    
    __get_new_program(){

        let this_ref = this;
        function ProgramInfo() {
            this.program = webglUtils.createProgramFromScripts(this_ref.gl, [this_ref.id_vrtx_shader, this_ref.id_frag_shader]);
            this.positionLocation = this_ref.gl.getAttribLocation(this.program, "a_position");
            this.colorLocation = this_ref.gl.getAttribLocation(this.program, "a_color");
            this.matrixLocation = this_ref.gl.getUniformLocation(this.program, "u_matrix");
            this.VertexNumber = 0;
            this.Vertex_buffer = null;
            this.Color_buffer = null;
            this.Strategy = null;
        }
        return new ProgramInfo();

    }

    __add_vertices(Vertices, color, strategy){

        let new_program = this.__get_new_program();
        new_program.VertexNumber = Vertices.length;
        new_program.Strategy = strategy;
        this.programs.push(new_program);

        let Colors = [];
        let ii = 0;
        for(let i=0; i<Vertices.length; i++){
            if(ii == 2){
                Colors.push(color[0]);
                Colors.push(color[1]);
                Colors.push(color[2]);
                Colors.push(color[3]);
                ii = 0;
            }
            else{
                ii++;
            }
        }

        new_program.Vertex_buffer = this.gl.createBuffer();
        if (!new_program.Vertex_buffer) {
            console.log("failed to create buffer");
            return;
        }
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, new_program.Vertex_buffer);
        this.gl.bufferData(this.gl.ARRAY_BUFFER, new Float32Array(Vertices), this.gl.STATIC_DRAW);
        
        new_program.Color_buffer = this.gl.createBuffer();
        if (!new_program.Color_buffer) {
            console.log("failed to create buffer");
            return;
        }
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, new_program.Color_buffer);
        this.gl.bufferData(this.gl.ARRAY_BUFFER, new Uint8Array(Colors), this.gl.STATIC_DRAW);
        
    };

    move_camera(Rotation , origin_distance){
        let C1 = Math.cos(Rotation[0]);
        let S1 = Math.sin(Rotation[0]);
        let C2 = Math.cos(Rotation[1]);
        let S2 = Math.sin(Rotation[1]);

        let R = [[C1, 0, S1],
                 [S1*S2, C2, -C1*S2],
                 [-S1*C2, S2, C1*C2]];
        let T = [origin_distance*R[0][2], origin_distance*R[1][2], origin_distance*R[2][2] ];

        this.mCamera_frame = [[R[0][0], R[0][1], R[0][2], 0],
                              [R[1][0], R[1][1], R[1][2], 0],
                              [R[2][0], R[2][1], R[2][2], 0],
                             [dot(T , get_col(R, 0)), dot(T , get_col(R, 1)), dot(T , get_col(R, 2)), 1]];
    };

    drawScene(){
        this.gl.clearColor(this.bckgrn[0], this.bckgrn[1], this.bckgrn[2], 1.0);
        this.gl.clear(this.gl.COLOR_BUFFER_BIT| this.gl.DEPTH_BUFFER_BIT);
        webglUtils.resizeCanvasToDisplaySize(this.gl.canvas);

        let matrix;
        for(let i=0; i<this.programs.length; i++){
            this.gl.useProgram(this.programs[i].program);
            this.gl.enableVertexAttribArray(this.programs[i].positionLocation);
            this.gl.enableVertexAttribArray(this.programs[i].colorLocation);
            this.gl.enableVertexAttribArray(this.programs[i].matrixLocation);

            this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.programs[i].Vertex_buffer);
            this.gl.vertexAttribPointer(this.programs[i].positionLocation, 3, this.gl.FLOAT, false, 0, 0);

            this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.programs[i].Color_buffer);
            this.gl.vertexAttribPointer(this.programs[i].colorLocation, 4, this.gl.UNSIGNED_BYTE, true, 0, 0);

            matrix = Mat_x_Mat(this.mCamera_frame, this.mProjection);

            function convert_matrix(mat){
                let m=[];
                let r;
                for(let i=0; i<mat.length; i++){
                    for(r =0; r<mat[i].length; r++){
                        m.push(mat[i][r]);
                    }
                }
                return m;
            }

            this.gl.uniformMatrix4fv(this.programs[i].matrixLocation, false, convert_matrix(matrix));
            this.gl.drawArrays( this.programs[i].Strategy, 0, this.programs[i].VertexNumber);
        }
    };

}

function get_col(M , c){
    let col = [];
    for(let i=0; i<M.length; i++){
        col.push(M[i][c]);
    }
    return col;
}

function dot(arr1, arr2){
    let d = 0;
    for(let i=0; i<arr1.length; i++){
        d += arr1[i]*arr2[i];
    }
    return d; 
}

function Mat_x_Mat(matA, matB){
    let result=[];
    let c;
    for(let r=0;r<matA.length; r++){
        result.push([]);
        for(c=0;c<matA.length; c++){
            result[r].push(   dot(matA[r], get_col(matB, c)) );
        }
    }
    return result;
}

function trasform(matrix, Point){
    let PP =[Point[0], Point[1], Point[2], 1];
    let P_new =[0,0,0];
    for(let i=0; i<3;i++){
        P_new[i] = dot(PP , get_col(matrix, i));
    }
    return P_new;
}
