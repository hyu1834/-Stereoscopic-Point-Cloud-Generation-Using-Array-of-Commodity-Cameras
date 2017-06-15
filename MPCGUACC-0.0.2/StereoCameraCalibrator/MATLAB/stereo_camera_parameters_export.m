function stereo_camera_parameters_export(filepath,...
                                         cameraMatrix1, distortionVector1,...
                                         cameraMatrix2, distortionVector2,...
                                         rotationMatrix, translationVector,...
                                         essentialMatrix, fundamentalMatrix)
    %open a file for exporting parameters
    fileOut = fopen(filepath, 'w');
    %Output XML version and specify this XML is for OpenCV
    fprintf(fileOut, '<?xml version="1.0"?>\n<opencv_storage>\n');
    
    %Output Camera1 Matrix - IntrinsicMatrix
    fprintf(fileOut, '<cameraMatrix1 type_id="opencv-matrix">\n');
    fprintf(fileOut, '\t<rows>3</rows>\n\t<cols>3</cols>\n\t<dt>d</dt>\n\t<data>\n\t');
    for i = 1 : 3
        for j = 1 : 3
            fprintf(fileOut, '%.16e ', cameraMatrix1(i, j));
        end
        fprintf(fileOut, '\n\t');
    end
    fprintf(fileOut, '</data>\n</cameraMatrix1>\n');
    
    %Output Camera1 Distorion Vector
    fprintf(fileOut, '<distortionVector1 type_id="opencv-matrix">\n');
    fprintf(fileOut, '\t<rows>1</rows>\n\t<cols>5</cols>\n\t<dt>d</dt>\n\t<data>\n\t');
    for i = 1 : 5
        fprintf(fileOut, '%.16e ', distortionVector1(i));
    end
    fprintf(fileOut, '</data>\n</distortionVector1>\n');
    
    %Output Camera2 Matrix - IntrinsicMatrix
    fprintf(fileOut, '<cameraMatrix2 type_id="opencv-matrix">\n');
    fprintf(fileOut, '\t<rows>3</rows>\n\t<cols>3</cols>\n\t<dt>d</dt>\n\t<data>\n\t');
    for i = 1 : 3
        for j = 1 : 3
            fprintf(fileOut, '%.16e ', cameraMatrix2(i, j));
        end
        fprintf(fileOut, '\n\t');
    end
    fprintf(fileOut, '</data>\n</cameraMatrix2>\n');
    
    %Output Camera2 Distorion Vector
    fprintf(fileOut, '<distortionVector2 type_id="opencv-matrix">\n');
    fprintf(fileOut, '\t<rows>1</rows>\n\t<cols>5</cols>\n\t<dt>d</dt>\n\t<data>\n\t');
    for i = 1 : 5
        fprintf(fileOut, '%.16e ', distortionVector2(i));
    end
    fprintf(fileOut, '</data>\n</distortionVector2>\n');
    
    %Output Rotation Matrix
    fprintf(fileOut, '<rotationMatrix type_id="opencv-matrix">\n');
    fprintf(fileOut, '\t<rows>3</rows>\n\t<cols>3</cols>\n\t<dt>d</dt>\n\t<data>\n\t');
    for i = 1 : 3
        for j = 1 : 3
            fprintf(fileOut, '%.16e ', rotationMatrix(i, j));
        end
        fprintf(fileOut, '\n\t');
    end
    fprintf(fileOut, '</data>\n</rotationMatrix>\n');

    %Output Translation Vector
    fprintf(fileOut, '<translationVector type_id="opencv-matrix">\n');
    fprintf(fileOut, '\t<rows>3</rows>\n\t<cols>1</cols>\n\t<dt>d</dt>\n\t<data>\n\t');
    for i = 1 : 3
        fprintf(fileOut, '%.16e ', translationVector(i));
    end
    fprintf(fileOut, '\n</data>\n</translationVector>\n');

    %Output Essential Matrix
    fprintf(fileOut, '<essentialMatrix type_id="opencv-matrix">\n');
    fprintf(fileOut, '\t<rows>3</rows>\n\t<cols>3</cols>\n\t<dt>d</dt>\n\t<data>\n\t');
    for i = 1 : 3
        for j = 1 : 3
            fprintf(fileOut, '%.16e ', essentialMatrix(i, j));
        end
        fprintf(fileOut, '\n\t');
    end
    fprintf(fileOut, '</data>\n</essentialMatrix>\n');

    %Output Fundamental Matrix
    fprintf(fileOut, '<fundamentalMatrix type_id="opencv-matrix">\n');
    fprintf(fileOut, '\t<rows>3</rows>\n\t<cols>3</cols>\n\t<dt>d</dt>\n\t<data>\n\t');
    for i = 1 : 3
        for j = 1 : 3
            fprintf(fileOut, '%.16e ', fundamentalMatrix(i, j));
        end
        fprintf(fileOut, '\n\t');
    end
    fprintf(fileOut, '</data>\n</fundamentalMatrix>\n');

    %Output closing flag for OpenCV storage
    fprintf(fileOut, '</opencv_storage>\n');
    
    fclose(fileOut);
end