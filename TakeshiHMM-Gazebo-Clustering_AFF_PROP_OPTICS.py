#!/usr/bin/env python
# coding: utf-8

# In[1]:


from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# In[2]:




class HMM (object):
             def __init__(self,A,B,PI):
                 self.A=A
                 self.B=B
                 self.PI=PI   
def viterbi(obs,Modelo1,PI):
    
    delta=np.zeros((len(obs)+1,len(Modelo1.A)))
    phi=np.zeros((len(obs)+1,len(A)))+666
    path =np.zeros(len(obs)+1)
    T=len(obs)
    Modelo1.PI = PI
    delta[0,:]= Modelo1.PI * Modelo1.B[:,obs[0]]
    phi[0,:]=666
    for t in range(len(obs)):
        for j in range(delta.shape[1]):

            delta [t+1,j]=np.max(delta[t] * A[:,j]) * B[j,obs[t]]
            phi[t+1,j]= np.argmax(delta[t] * A[:,j])
    path[T]=int(np.argmax(delta[T,:]))
    for i in np.arange(T-1,0,-1):
        #print (i,phi[i+1,int(path[i+1])])
        path[i]=phi[i+1,int(path[i+1])]
    return(path)
def cuantizar_xy(xy, cc):
    xycuant=cc
    out=np.power(xycuant-xy,2).sum(axis=1).argmin()
    return out
    
def path_to_xy(path,ccxy):
    estimated= pd.DataFrame(path.astype(int).T)
    estimated.columns=[['Path_vit']]
    estimated['xcuant'] = estimated['Path_vit'].apply(lambda x: ccxy[x,0])
    estimated['ycuant'] = estimated['Path_vit'].apply(lambda x: ccxy[x,1])                                
    return (estimated)
def quantized(xyth,ccxyth):
    xythcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))
    x,y=ccxyth[xythcuant,:2]
    return ((x,y),(xythcuant))


# In[3]:


def Markov_A_2_grafo(A,ccxyth):
    dists=np.zeros(A.shape)
    for i in range(A.shape[0]):
        for j in range (A.shape[1]):
            if A[i,j]!=0 :
                dists[i,j]= np.linalg.norm(ccxyth[i]-ccxyth[j])    
    
    
    con = np.where(dists==0,np.inf,dists)
    graphe2=grafo(ccxyth,con)
    return graphe2


class node(object):
    def __init__(self,x,y):
        self.x=x
        self.y=y
        
        
class grafo (object):
             def __init__(self,nodos,conec):
                 self.nodos=nodos
                 self.conec=conec        

def dijkstra(nodoinicial,nodofinal,graphe):
    

    numnodos= len(graphe.nodos)
    con = graphe.conec
    D= np.ones(numnodos)*np.inf
    Prv= np.ones(numnodos)*np.inf
    V= np.zeros(numnodos)
    a = nodoinicial
    D[a]=0
    Prv[a]=0
    Prv[np.where(con[a]!=np.inf)]=a
    V[a]=1
    Dacc=D[a]
    ########
    D=np.minimum(D,con[a]+D[a])
    cont=0
    sucess=False
    while(sucess==False):
        a = np.argmin(D+np.where (V==1,np.inf, V))
        Dacc=D[a]
        Prv[np.where(D>(con[a]+Dacc) )]=a
        V[a]=1
        D=np.minimum(D,con[a]+Dacc)
        if (a== nodofinal):
            print("RUTA CALCULADA ")
            sucess=True
    rutainv=[]
    rutainv.append(nodofinal)
    while(rutainv[-1]!=nodoinicial):
        prv=Prv[int(rutainv[-1])]
        rutainv.append(prv)

    ruta=[]
    for n in reversed(rutainv):
        ruta.append(n)
    return(ruta)


# In[4]:


fast_load=True
get_new_ccs= False


if fast_load:
    datamugroso=pd.read_csv('pddata.csv')
    data= datamugroso.iloc[:,1:].sample(n=9000,axis=0)
    #lecs=data[data.Vk==lec_leida].sample(n=10,axis=0).iloc[:,:-6]

lecs=data.iloc[:,:-6]


# In[5]:


data.describe()


# In[6]:


lecs=np.clip(lecs,0,5)
data.iloc[:,:-6]=lecs


# In[7]:


data.describe()


# In[8]:


from sklearn.cluster import AffinityPropagation
from sklearn.cluster import DBSCAN


# In[9]:


af = AffinityPropagation(damping=.8).fit(lecs)
#db=DBSCAN().fit(lecs)


# In[10]:


data['Vk_aff']=af.labels_
ccvk_aff= af.cluster_centers_


# In[11]:


data[['Vk','Vk_aff']]


# In[12]:


ccvk=np.load('ccvk.npy')
fig = plt.figure(figsize=(10,10))

ax1 = fig.add_subplot(111)
ax1.scatter(x= data.x, y=data.y,marker='+',c=data.Vk_aff,alpha=.313)
fig.suptitle('Affinity Prop', fontsize=16)




#cords3=path_to_xy(np.arange(0,len(cc)),cc)
#ax1.scatter(x= cords3.xcuant, y=cords3.ycuant ,marker='.',s=40,c='g')



# In[13]:


data.Vk_aff.value_counts().index[1] 


# In[14]:


#index_forlec=data.Vk_aff.value_counts().iloc[1]
lec_aff_leida= data.Vk_aff.value_counts().index[1]
lec=data[data.Vk_aff==lec_aff_leida].iloc[0,:-6].values

#lec_Kmn_leida= data.Vk.value_counts().index[1]

#lec=data.iloc[index_forlec,:-6].values
lec_Kmn_leida=np.power(lec.T-ccvk,2).sum(axis=1,keepdims=True).argmin()
lec_Kmn_leida,lec_aff_leida


# In[15]:


#lec=np.load('lec.npy')

start_ang=-240/2*np.pi/180
stop_ang= 240/2*np.pi/180
angs=np.linspace(start_ang,stop_ang,num=len(lec))


# In[17]:


#ccvk=cc
cc=ccvk
cordsy, cordsx= np.cos(angs) *  lec ,  np.sin(angs) *  lec
CORDS=pd.DataFrame((cordsx,cordsy))
CORDS=CORDS.T
CORDS.columns=['x','y']
#print(np.power(lec.T-ccvk,2).sum(axis=1,keepdims=True).argmax())
#vec_obs=ccvk[np.power(lec.T-ccvk,2).sum(axis=1,keepdims=True).argmax()]
vec_obs_aff=ccvk_aff[lec_aff_leida]
vec_obs=cc[lec_Kmn_leida]

cordsy, cordsx= np.cos(angs) *  vec_obs ,  np.sin(angs) *  vec_obs_aff
CORDS['xx'],CORDS['yy']=cordsx,cordsy
cordsy, cordsx= np.cos(angs) *  vec_obs_aff ,  np.sin(angs) *  vec_obs_aff
CORDS['xxx'],CORDS['yyy']=cordsx,cordsy

fig = plt.figure(figsize=(10,10))
fig.suptitle('Comparison Between real read and cuantized', fontsize=16)

ax1 = fig.add_subplot(111)
ax1.scatter(x= CORDS.x, y=CORDS.y,marker='+' , label= 'Real Read')
ax1.scatter(x=CORDS.xx,y=CORDS.yy,marker='*', label= 'MiniKmeans Centroid')
ax1.scatter(x=CORDS.xxx,y=CORDS.yyy,marker='|', label= 'Affinity Prop')
ax1.legend()


# In[18]:


lec_aff_leida= data.Vk_aff.value_counts().index[25]
print (lec_aff_leida)


# In[19]:


#index_forlec=data.Vk_aff.value_counts().iloc[1]
lec=data[data.Vk_aff==lec_aff_leida].iloc[0,:-6].values

lec_Kmn_leida= np.power(lec.T-ccvk,2).sum(axis=1,keepdims=True).argmin()

#lec=data.iloc[index_forlec,:-6].values

#vec_obs=ccvk[np.power(lec.T-ccvk,2).sum(axis=1,keepdims=True).argmin()]

print(lec_aff_leida, lec_Kmn_leida)


# In[20]:


#lec=np.load('lec.npy')

start_ang=-240/2*np.pi/180
stop_ang= 240/2*np.pi/180
angs=np.linspace(start_ang,stop_ang,num=len(lec))


# In[21]:


#ccvk=cc
cordsy, cordsx= np.cos(angs) *  lec ,  np.sin(angs) *  lec
CORDS=pd.DataFrame((cordsx,cordsy))
CORDS=CORDS.T
CORDS.columns=['x','y']
#print(np.power(lec.T-ccvk,2).sum(axis=1,keepdims=True).argmax())
#vec_obs=ccvk[np.power(lec.T-ccvk,2).sum(axis=1,keepdims=True).argmax()]
vec_obs_aff=ccvk_aff[lec_aff_leida]
vec_obs=cc[lec_Kmn_leida]

cordsy, cordsx= np.cos(angs) *  vec_obs ,  np.sin(angs) *  vec_obs_aff
CORDS['xx'],CORDS['yy']=cordsx,cordsy
cordsy, cordsx= np.cos(angs) *  vec_obs_aff ,  np.sin(angs) *  vec_obs_aff
CORDS['xxx'],CORDS['yyy']=cordsx,cordsy

fig = plt.figure(figsize=(10,10))
fig.suptitle('Comparison Between real read and cuantized', fontsize=16)

ax1 = fig.add_subplot(111)
ax1.scatter(x= CORDS.x, y=CORDS.y,marker='+' , label= 'Real Read')
ax1.scatter(x=CORDS.xx,y=CORDS.yy,marker='*', label= 'MiniKmeans Centroid')
ax1.scatter(x=CORDS.xxx,y=CORDS.yyy,marker='|', label= 'Affinity Prop')
ax1.legend()


# In[22]:


from joblib import dump, load
dump(af, 'aff_prop_class.joblib_2',protocol=2) 


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:


lecs=data.iloc[:,:-6]
from sklearn.cluster import OPTICS
clust = OPTICS(min_samples=10)


# In[ ]:


#####clust.fit(lecs)###NO PREDICT FUNC SO UNFEASIBLE$


# In[ ]:


pd.Series(clust.labels_).describe()
data.Vk_aff=clust.labels_


# In[ ]:


dump(clust, 'OPTICS_class.joblib',protocol=2) 


# In[ ]:


#index_forlec=data.Vk_aff.value_counts().iloc[1]
lec_leida= data.Vk_aff.value_counts().index[1]
#lec=data.iloc[index_forlec,:-6].values
lec=data[data.Vk_aff==lec_leida].iloc[0,:-6].values
lec_leida


# In[ ]:





# In[ ]:


#lec=np.load('lec.npy')

start_ang=-240/2*np.pi/180
stop_ang= 240/2*np.pi/180
angs=np.linspace(start_ang,stop_ang,num=len(lec))


# In[ ]:





# In[ ]:


auxy , auxx =np.array([0,0]),np.array([0,0])
start_ang=-250/2*np.pi/180
stop_ang= 250/2*np.pi/180
angs=np.linspace(start_ang,stop_ang,num=len(vec_obs))


lecs=data[data.Vk_aff==lec_leida].sample(n=10,axis=0).iloc[:,:-6]

for  vec_obs in lecs.values.tolist():
    cordsy,cordsx= np.cos(angs) *  vec_obs ,  np.sin(angs) *  vec_obs
    auxy,auxx=np.concatenate((auxy,cordsy)),np.concatenate((auxx,cordsx))
print(lec_leida)    


# In[ ]:



CORDS=pd.DataFrame((auxx,auxy))
CORDS=CORDS.T
CORDS.columns=[['x','y']]

vec_obs=ccvk_aff[lec_leida]
cordsy, cordsx= np.cos(angs) *  vec_obs ,  np.sin(angs) *  vec_obs

CORDSVK=pd.DataFrame()
CORDSVK['xx'],CORDSVK['yy']=cordsx,cordsy


fig = plt.figure(figsize=(10,10))
fig.suptitle('Comparison Between real 15 random reads and their cuantized versions OPTICS', fontsize=16)

ax1 = fig.add_subplot(111)
ax1.scatter(x= CORDS.x, y=CORDS.y,marker='+')
#ax1.scatter(x=CORDSVK.xx,y=CORDSVK.yy,marker='*')
print(lec_leida)


# In[ ]:


lec_leida= data.Vk_aff.value_counts().index[3]

lec=data[data.Vk_aff==lec_leida].iloc[0,:-6].values

lec_leida


# In[ ]:


ccvk=cc
cordsy, cordsx= np.cos(angs) *  lec ,  np.sin(angs) *  lec
CORDS=pd.DataFrame((cordsx,cordsy))
CORDS=CORDS.T
CORDS.columns=[['x','y']]
#print(np.power(lec.T-ccvk,2).sum(axis=1,keepdims=True).argmax())
vec_obs=ccvk[np.power(lec.T-ccvk,2).sum(axis=1,keepdims=True).argmax()]
vec_obs_aff=ccvk_aff[lec_leida]

cordsy, cordsx= np.cos(angs) *  vec_obs ,  np.sin(angs) *  vec_obs
CORDS['xx'],CORDS['yy']=cordsx,cordsy
cordsy, cordsx= np.cos(angs) *  vec_obs_aff ,  np.sin(angs) *  vec_obs_aff
CORDS['xxx'],CORDS['yyy']=cordsx,cordsy

fig = plt.figure(figsize=(10,10))
fig.suptitle('Comparison Between real read and cuantized', fontsize=16)

ax1 = fig.add_subplot(111)
ax1.scatter(x= CORDS.x, y=CORDS.y,marker='+' , label= 'Real Read')
#ax1.scatter(x=CORDS.xx,y=CORDS.yy,marker='*', label= 'MiniKmeans Centroid')
ax1.scatter(x=CORDS.xxx,y=CORDS.yyy,marker='|', label= 'OPTICS')
ax1.legend()


# In[ ]:


auxy , auxx =np.array([0,0]),np.array([0,0])
start_ang=-250/2*np.pi/180
stop_ang= 250/2*np.pi/180
angs=np.linspace(start_ang,stop_ang,num=len(vec_obs))


lecs=data[data.Vk_aff==lec_leida].sample(n=10,axis=0).iloc[:,:-6]

for  vec_obs in lecs.values.tolist():
    cordsy,cordsx= np.cos(angs) *  vec_obs ,  np.sin(angs) *  vec_obs
    auxy,auxx=np.concatenate((auxy,cordsy)),np.concatenate((auxx,cordsx))
print(lec_leida)    


# In[ ]:



CORDS=pd.DataFrame((auxx,auxy))
CORDS=CORDS.T
CORDS.columns=[['x','y']]

vec_obs=ccvk_aff[lec_leida]
cordsy, cordsx= np.cos(angs) *  vec_obs ,  np.sin(angs) *  vec_obs

CORDSVK=pd.DataFrame()
CORDSVK['xx'],CORDSVK['yy']=cordsx,cordsy


fig = plt.figure(figsize=(10,10))
fig.suptitle('Comparison Between real 15 random reads and their cuantized versions OPTICS', fontsize=16)

ax1 = fig.add_subplot(111)
ax1.scatter(x= CORDS.x, y=CORDS.y,marker='+')
ax1.scatter(x=CORDSVK.xx,y=CORDSVK.yy,marker='*')
print(lec_leida)


# In[ ]:


dump(af, 'aff_prop_class.joblib',protocol=2) 


# In[ ]:


lecs=data.iloc[:,:-6]
from sklearn.cluster import FeatureAgglomeration
agglo = FeatureAgglomeration(n_clusters=15)


# In[ ]:


agglo.fit(lecs)


# In[ ]:


lecs_reduced=agglo.fit_transform(lecs)


# In[ ]:


lecs_reduced.shape


# In[ ]:


from sklearn.decomposition import PCA, KernelPCA


# In[ ]:


pca =PCA(n_components=15)
pca.fit(lecs)


# In[ ]:


pca.transform(lecs).shape


# In[ ]:


###TO DO GRID SEARCH PCA. FEATURE AGGL


# In[ ]:





# In[ ]:


##LBG----- REading Doc's centroids
f=open('vq_images_sonar_32.dat')
contents=f.readlines()
cc_lgbt=[]
for char in contents:
    words=char.split()
    cc_lgbt.append(words[2:])
cc_buzo=[]
for i in range(32):
    cc_buzo.append(np.asarray(cc_lgbt[i]))
cc_buzo=np.asarray(cc_buzo)


# In[ ]:


cc_buzo.shape


# In[ ]:





# In[ ]:


len(cc_buzo)


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:


clust


# In[ ]:





# In[ ]:




