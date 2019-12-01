let Data;

function get_loader(after_load_fnct){

    let uploader = document.createElement("input");
    uploader.setAttribute("type", "file");
    uploader.onchange = () => {
        let file = uploader.files[0];
        if (file) {
            const reader = new FileReader();
            reader.readAsText(file, 'UTF-8');
            reader.onload = (evt) => {
                Data = JSON.parse(evt.target.result);
                console.log(Data);
                let Size = center_data();

                after_load_fnct(Data, Size);
            };
            reader.onerror = (evt) => {
                console.error('Failed to load the file');
            };
        }
    };
    return uploader;

}

function center_data(){
    function get_min_max(axis){
        let support = [0,0];
        for(let i=0; i<Data.length; i++){
            if(Data[i][axis] < support[0]){
                support[0] = Data[i][axis];
            }
            if(Data[i][axis] > support[1]){
                support[1] = Data[i][axis];
            }
        }
        return support;
    }

    //compute AABB
    let support_x=get_min_max(0);
    let support_y=get_min_max(1);
    let support_z=get_min_max(2);
    
    //compute AABB center
    let center=[0,0,0];
    center[0] = 0.5 * (support_x[0] + support_x[1] );
    center[1] = 0.5 * (support_y[0] + support_y[1] );
    center[2] = 0.5 * (support_z[0] + support_z[1] );

    let ii;
    for(let i=0; i<Data.length; i++){
        for(ii=0;ii<3; ii++){
            Data[i][ii] -= center[ii];
        }
    }

    //return 
    let Size = support_x[1] - support_x[0];
    let Size_temp = support_y[1] - support_y[0];
    if(Size_temp > Size) {
        Size = Size_temp;
    }
    Size_temp = support_z[1] - support_z[0];
    if(Size_temp > Size) {
        Size = Size_temp;
    }
    return Size;
}