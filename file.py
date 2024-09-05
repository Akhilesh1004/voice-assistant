
import os
import time
import urllib.request

list = {"photo_american_shorthair", "photo_bengal_cat", "photo_british_shorthair", "photo_maine_coon",
        "photo_munchkin_cat", "photo_norwegian_forest_cat", "photo_persian_cat", "photo_ragdoll_cat", "photo_russian_blue_cat"
        , "photo_scottish_fold_cat", "photo_siamese_cat", "photo_snowshoe_cat", "photo_sphynx_cat"}


folder_path ='./photo_british_shorthair/'
if (os.path.exists(folder_path) == False): #判斷資料夾是否存在
    os.makedirs(folder_path) #Create folder
m = 0
for i in len(list):
    path = folder_path+list[i]
    allFileList = os.listdir(path)
    for file in allFileList:
        print(m)
        # 保存圖片
        m += 1
        filename = list[i]+str(m) + '.png'
        urllib.request.urlretrieve(img_url, os.path.join(folder_path , filename))
print('Done')

